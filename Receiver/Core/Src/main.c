/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : IBC receiver (BFSK/OOK)
 ******************************************************************************
 *
 * ADC 200 kHz -> Goertzel (10/20 kHz + 5 kHz noise bin) -> edge-tracking
 * bit sync, centre-3 vote -> Barker-13 + PRBS-9 (BER) or CRC frames (packet).
 *
 * BER_TEST_MODE: 1 = BER stream, 0 = packet mode. BPS_MODE 20/40/100,
 * MOD_MODE 0 = BFSK / 1 = OOK. Same values on TX.
 *
 * NUCLEO-F411RE @ 100 MHz. ADC1 ch10 (PC0) -> signal electrode, GND ->
 * return electrode. TIM3 200 kHz ADC trigger, USART2 log.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* non-blocking UART log: ring buffer drained from the main loop */
#define TXR_SIZE 2048u
static uint8_t  txr_buf[TXR_SIZE];
static volatile uint16_t txr_head = 0;
static volatile uint16_t txr_tail = 0;
static volatile uint8_t  tx_async = 0;   /* 0 = blocking, 1 = buffered */

static inline void txr_push(uint8_t c)
{
    uint16_t next = (uint16_t)((txr_head + 1u) % TXR_SIZE);
    if (next != txr_tail) { txr_buf[txr_head] = c; txr_head = next; }
}

static void txr_drain(void)
{
    while (txr_tail != txr_head && __HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE))
    {
        huart2.Instance->DR = txr_buf[txr_tail];
        txr_tail = (uint16_t)((txr_tail + 1u) % TXR_SIZE);
    }
}

static void txr_flush_blocking(void)
{
    while (txr_tail != txr_head)
    {
        if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE))
        {
            huart2.Instance->DR = txr_buf[txr_tail];
            txr_tail = (uint16_t)((txr_tail + 1u) % TXR_SIZE);
        }
    }
}

int __io_putchar(int ch)
{
    uint8_t c = (uint8_t)ch;
    if (tx_async) {
        txr_push(c);
    } else {
        HAL_UART_Transmit(&huart2, &c, 1, 10);
    }
    return ch;
}

/* 1 = BER stream, 0 = packet mode */
#define BER_TEST_MODE   0

/* 1 = idle power bench (ADC/TIM3 off, WFI; measure at JP6, no debugger) */
#ifndef POWER_IDLE_TEST
#define POWER_IDLE_TEST 0
#endif

/* bit rate: 20, 40 or 100 bps (same on TX); window N = FS_ADC/(5*bps) */
#ifndef BPS_MODE
#define BPS_MODE 20
#endif

/* 0 = BFSK (10/20 kHz), 1 = OOK (20 kHz vs adaptive threshold) */
#ifndef MOD_MODE
#define MOD_MODE 1
#endif
#if MOD_MODE != 0 && MOD_MODE != 1
#error "MOD_MODE must be 0 (BFSK) or 1 (OOK)"
#endif

/* auto-stop N minutes after the first sync (0 = run forever) */
#ifndef TEST_DURATION_MIN
#define TEST_DURATION_MIN 25u
#endif
#define TEST_DURATION_MS  ((uint32_t)TEST_DURATION_MIN * 60000u)

#define FS_ADC       200000u

#if BPS_MODE == 20
  #define HALF_SAMPLES 2000u
#elif BPS_MODE == 40
  #define HALF_SAMPLES 1000u
#elif BPS_MODE == 100
  #define HALF_SAMPLES 400u
#else
  #error "Unsupported BPS_MODE: choose 20, 40 or 100"
#endif

#define N_SAMPLES    (HALF_SAMPLES * 2u)
#define F_NOISE      5000.0f   /* noise bin */

static uint16_t adc_buf[N_SAMPLES];
static volatile uint32_t adc_half_ready = 0;

typedef struct { float coeff; } goertzel_cfg_t;
static goertzel_cfg_t g10, g20, g_noise;

static void goertzel_init(goertzel_cfg_t *g, uint32_t N, float fs, float f0)
{
    float kf = (N * f0) / fs;
    uint32_t k = (uint32_t)(kf + 0.5f);
    float w = 2.0f * 3.14159265f * (float)k / (float)N;
    g->coeff = 2.0f * cosf(w);
}

/* single-bin power, DC removed */
static float goertzel_power(const goertzel_cfg_t *g, const uint16_t *x, uint32_t N)
{
    uint32_t sum = 0;
    for (uint32_t i = 0; i < N; i++) sum += x[i];
    float mean = (float)sum / (float)N;

    float s0 = 0.0f, s1 = 0.0f, s2 = 0.0f;
    float coeff = g->coeff;
    for (uint32_t i = 0; i < N; i++) {
        float v = (float)x[i] - mean;
        s0 = v + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }
    return (s1 * s1 + s2 * s2 - coeff * s1 * s2);
}

static uint8_t last_bit = 1;

#if MOD_MODE == 0
#define MIN_POWER_THRESHOLD  1e4f
#define DIFF_FRACTION  0.15f

static uint8_t classify_bfsk(float p10, float p20)
{
    if (!(p10 == p10) || p10 < 0) p10 = 0;
    if (!(p20 == p20) || p20 < 0) p20 = 0;

    float pmax = (p10 > p20) ? p10 : p20;
    if (pmax < MIN_POWER_THRESHOLD) return last_bit;

    float diff = p20 - p10;
    float margin = pmax * DIFF_FRACTION;

    if (diff > margin)       last_bit = 1;
    else if (diff < -margin) last_bit = 0;

    return last_bit;
}
#endif /* MOD_MODE == 0 */

#if MOD_MODE == 1
/* OOK threshold: geometric mean of on/off EMAs, hysteresis, noise bootstrap */
#define OOK_NOISE_FACTOR  10.0f
#define OOK_EMA_ALPHA     0.05f
#define OOK_HYST          1.5f

static float ook_ema_on = 0.0f, ook_ema_off = 0.0f;

static uint8_t classify_ook(float pc, float p_noise)
{
    if (!(pc == pc) || pc < 0) pc = 0;
    if (!(p_noise == p_noise) || p_noise < 1.0f) p_noise = 1.0f;

    float thr;
    if (ook_ema_on > 0.0f && ook_ema_off > 0.0f)
        thr = sqrtf(ook_ema_on * ook_ema_off);
    else
        thr = OOK_NOISE_FACTOR * p_noise;

    if (pc > thr * OOK_HYST)      last_bit = 1;
    else if (pc * OOK_HYST < thr) last_bit = 0;
    /* else keep last_bit */

    if (last_bit)
        ook_ema_on  = (ook_ema_on  <= 0.0f) ? pc
                      : ook_ema_on  + OOK_EMA_ALPHA * (pc - ook_ema_on);
    else
        ook_ema_off = (ook_ema_off <= 0.0f) ? pc
                      : ook_ema_off + OOK_EMA_ALPHA * (pc - ook_ema_off);

    return last_bit;
}
#endif /* MOD_MODE == 1 */

#if BER_TEST_MODE
/* stream: 64-bit preamble + Barker-13 + 511-bit PRBS-9, vs local replica */

static uint16_t prbs9_state_rx = 0x1FFu;
static uint8_t prbs9_next_bit_rx(void)
{
    uint8_t out = (uint8_t)(((prbs9_state_rx >> 8) ^ (prbs9_state_rx >> 4)) & 1u);
    prbs9_state_rx = (uint16_t)(((prbs9_state_rx << 1) | out) & 0x1FFu);
    return out;
}

#define BER_BARKER_LEN     13u
#define BER_BARKER_PATTERN 0x1F35u    /* 1111100110101 */
#define BER_BARKER_MASK    0x1FFFu
#define BER_PRBS_BITS      511u

/* gate: the 8 bits before the Barker must be 0x55 (fewer false syncs) */
#define BER_PREAMBLE_CHECK_BITS 8u
#define BER_PREAMBLE_CHECK_MASK 0xFFu
#define BER_PREAMBLE_EXPECT     0x55u

/* seq BER >= threshold -> slip, excluded from BER_cond */
#define BER_VALID_THRESHOLD     0.10f

typedef enum { BER_SEARCHING = 0, BER_SYNCED } ber_state_t;
static ber_state_t ber_st        = BER_SEARCHING;
static uint32_t    ber_shifter   = 0;
static uint32_t    ber_seq_idx   = 0;

static uint32_t ber_seq_total = 0;
static uint32_t ber_seq_err   = 0;

/* cum = all sequences, valid = lock-only */
static uint32_t ber_cum_total  = 0;
static uint32_t ber_cum_err    = 0;
static uint32_t ber_sync_count = 0;

static uint32_t ber_valid_syncs = 0;
static uint32_t ber_slip_syncs  = 0;
static uint32_t ber_valid_bits  = 0;
static uint32_t ber_valid_errs  = 0;

static uint32_t test_start_ms = 0;
static uint8_t  test_done     = 0;

static void ber_process_bit(uint8_t b)
{
    ber_shifter = (ber_shifter << 1) | (b & 1u);

    if (ber_st == BER_SEARCHING) {
        uint16_t barker = (uint16_t)(ber_shifter & BER_BARKER_MASK);
        uint8_t  prefix = (uint8_t)((ber_shifter >> BER_BARKER_LEN) & BER_PREAMBLE_CHECK_MASK);
        if (barker == BER_BARKER_PATTERN && prefix == BER_PREAMBLE_EXPECT) {
            prbs9_state_rx = 0x1FFu;
            ber_st         = BER_SYNCED;
            ber_seq_idx    = 0;
            ber_seq_total  = 0;
            ber_seq_err    = 0;
            ber_sync_count++;
            if (test_start_ms == 0) {
                test_start_ms = HAL_GetTick();
            }
            printf("BER: SYNC #%lu (gated)\r\n", (unsigned long)ber_sync_count);
        }
        return;
    }

    uint8_t expected = prbs9_next_bit_rx();
    if (b != expected) {
        ber_seq_err++;
        ber_cum_err++;
    }
    ber_seq_total++;
    ber_cum_total++;
    ber_seq_idx++;

    if (ber_seq_idx >= BER_PRBS_BITS) {
        float ber_seq  = (ber_seq_total  > 0) ? (float)ber_seq_err  / (float)ber_seq_total  : 0.0f;
        float ber_cum  = (ber_cum_total  > 0) ? (float)ber_cum_err  / (float)ber_cum_total  : 0.0f;
        const char *tag;
        if (ber_seq < BER_VALID_THRESHOLD) {
            ber_valid_syncs++;
            ber_valid_bits += ber_seq_total;
            ber_valid_errs += ber_seq_err;
            tag = "LOCK";
        } else {
            ber_slip_syncs++;
            tag = "SLIP";
        }
        float ber_cond = (ber_valid_bits > 0)
                         ? (float)ber_valid_errs / (float)ber_valid_bits : 0.0f;
        printf("BER: END_SEQ #%lu [%s] seq_err=%lu BER_seq=%.4e | "
               "lock=%lu slip=%lu BER_cond=%.4e BER_cum=%.4e\r\n",
               (unsigned long)ber_sync_count, tag,
               (unsigned long)ber_seq_err, (double)ber_seq,
               (unsigned long)ber_valid_syncs, (unsigned long)ber_slip_syncs,
               (double)ber_cond, (double)ber_cum);
        ber_st      = BER_SEARCHING;
        ber_seq_idx = 0;
        /* keep ber_shifter: the next sync shifts in naturally */
    }
}
#endif /* BER_TEST_MODE */

/* frame: 0x55 preamble, sync 0xD3 0x91, LEN, SEQ, payload, CRC-16/CCITT-FALSE LE */
#define PREAMBLE_BYTE      0x55u
#define PREAMBLE_MIN       4u
#define SYNC1_BYTE         0xD3u
#define SYNC2_BYTE         0x91u
#define FRAME_PAYLOAD_MAX  80u

static uint16_t crc16_ccitt_false_update(uint16_t crc, uint8_t data)
{
    crc ^= (uint16_t)data << 8;
    for (int i = 0; i < 8; i++) {
        if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
        else              crc = (crc << 1);
    }
    return crc;
}

typedef enum {
    FP_SEARCH = 0, FP_SYNC2, FP_LEN, FP_SEQ,
    FP_PAYLOAD, FP_CRC_LO, FP_CRC_HI
} fp_state_t;

static struct {
    fp_state_t st;
    uint8_t pre_cnt, len, seq;
    uint8_t payload[FRAME_PAYLOAD_MAX];
    uint8_t idx;
    uint16_t crc_rx, crc_calc;
} fp;

static void fp_reset(void) {
    fp.st = FP_SEARCH; fp.pre_cnt = 0; fp.len = 0; fp.seq = 0;
    fp.idx = 0; fp.crc_rx = 0; fp.crc_calc = 0xFFFF;
}
static void fp_init(void) { fp_reset(); }

static uint32_t fp_t0 = 0;
#define FP_TIMEOUT_MS 5000u
static inline void fp_note_activity(void) { fp_t0 = HAL_GetTick(); }

static uint32_t stat_ok = 0, stat_crc_fail = 0, stat_gaps = 0;
static uint8_t have_seq = 0, last_seq_ok = 0;

static float p10_acc = 0.0f, p20_acc = 0.0f, p_noise_acc = 0.0f;
static uint32_t p_count = 0;

static void fp_on_frame_ok(uint8_t len, uint8_t seq, const uint8_t *pl)
{
    stat_ok++;

    printf("FRAME OK seq=%u len=%u", (unsigned)seq, (unsigned)len);
    if (len == 2u) {
        /* uint16 LE, mg/dL */
        uint16_t val = (uint16_t)pl[0] | ((uint16_t)pl[1] << 8);
        printf(" %u mg/dL", (unsigned)val);
    } else {
        char s[FRAME_PAYLOAD_MAX + 1] = {0};
        uint8_t n = (len > FRAME_PAYLOAD_MAX) ? FRAME_PAYLOAD_MAX : len;
        for (uint8_t i = 0; i < n; i++) {
            uint8_t c = pl[i];
            if (c < 32 || c > 126) { s[0] = '\0'; break; }
            s[i] = (char)c;
            s[i + 1] = '\0';
        }
        if (s[0]) printf(" MSG: %s", s);
    }
    printf(" [ok=%lu fail=%lu gaps=%lu]\r\n",
           (unsigned long)stat_ok, (unsigned long)stat_crc_fail, (unsigned long)stat_gaps);

    if (have_seq) {
        uint8_t exp = (uint8_t)(last_seq_ok + 1u);
        if (seq != exp) {
            stat_gaps++;
            printf("SEQ GAP %u->%u\r\n", (unsigned)last_seq_ok, (unsigned)seq);
        }
    } else {
        have_seq = 1;
    }
    last_seq_ok = seq;

    /* PKT,<t_ms>,<seq>,<mgdl>,<ok>,<crc_fail>,<gaps> */
    if (len == 2u) {
        uint16_t val = (uint16_t)pl[0] | ((uint16_t)pl[1] << 8);
        printf("PKT,%lu,%u,%u,%lu,%lu,%lu\r\n",
               (unsigned long)HAL_GetTick(), (unsigned)seq, (unsigned)val,
               (unsigned long)stat_ok, (unsigned long)stat_crc_fail,
               (unsigned long)stat_gaps);
    }
}

static void fp_push_byte(uint8_t b)
{
    fp_note_activity();
    switch (fp.st) {
    case FP_SEARCH:
        if (b == PREAMBLE_BYTE) { if (fp.pre_cnt < 255) fp.pre_cnt++; return; }
        if (fp.pre_cnt >= PREAMBLE_MIN && b == SYNC1_BYTE) { fp.st = FP_SYNC2; return; }
        fp.pre_cnt = 0;
        return;
    case FP_SYNC2:
        if (b == SYNC2_BYTE) fp.st = FP_LEN; else fp_reset();
        return;
    case FP_LEN:
        fp.len = b;
        if (fp.len > FRAME_PAYLOAD_MAX) { fp_reset(); return; }
        fp.crc_calc = 0xFFFF;
        fp.crc_calc = crc16_ccitt_false_update(fp.crc_calc, fp.len);
        fp.st = FP_SEQ;
        return;
    case FP_SEQ:
        fp.seq = b;
        fp.crc_calc = crc16_ccitt_false_update(fp.crc_calc, fp.seq);
        fp.idx = 0;
        fp.st = (fp.len == 0) ? FP_CRC_LO : FP_PAYLOAD;
        return;
    case FP_PAYLOAD:
        fp.payload[fp.idx++] = b;
        fp.crc_calc = crc16_ccitt_false_update(fp.crc_calc, b);
        if (fp.idx >= fp.len) fp.st = FP_CRC_LO;
        return;
    case FP_CRC_LO:
        fp.crc_rx = (uint16_t)b;
        fp.st = FP_CRC_HI;
        return;
    case FP_CRC_HI:
        fp.crc_rx |= ((uint16_t)b << 8);
        if (fp.crc_rx == fp.crc_calc) {
            fp_on_frame_ok(fp.len, fp.seq, fp.payload);
        } else {
            stat_crc_fail++;
            printf("CRC FAIL rx=%04X calc=%04X [fail=%lu]\r\n",
                   (unsigned)fp.crc_rx, (unsigned)fp.crc_calc, (unsigned long)stat_crc_fail);
            /* PKTFAIL,<t_ms>,<seq>,<ok>,<crc_fail> */
            printf("PKTFAIL,%lu,%u,%lu,%lu\r\n",
                   (unsigned long)HAL_GetTick(), (unsigned)fp.seq,
                   (unsigned long)stat_ok, (unsigned long)stat_crc_fail);
        }
        fp_reset();
        return;
    }
}

/* UART-like deframer: start, 8 data LSB-first, stop */
typedef enum { RX_WAIT_START = 0, RX_BITS, RX_STOP } rx_state_t;
static rx_state_t rx_st = RX_WAIT_START;
static uint8_t rx_byte = 0, rx_bit_i = 0;

#define WINS_PER_BIT  5u

static uint8_t win_buf[WINS_PER_BIT];
static uint8_t bit_cnt = 0;
static uint8_t start_zeros = 0, stop_ones = 0, idle_ones = 0;
static uint8_t prev_raw = 1;
static uint8_t synced = 0;

static void rx_soft_reset(void)
{
    rx_st = RX_WAIT_START;
    rx_byte = 0; rx_bit_i = 0;
    bit_cnt = 0;
    start_zeros = 0; stop_ones = 0; idle_ones = 0;
    prev_raw = 1;
    synced = 0;
    for (int i = 0; i < (int)WINS_PER_BIT; i++) win_buf[i] = 1;
}

static uint8_t center_vote(void)
{
    uint8_t ones = win_buf[1] + win_buf[2] + win_buf[3];
    return (ones >= 2) ? 1u : 0u;
}

static void rx_push_bit(uint8_t b)
{
    switch (rx_st) {
    case RX_WAIT_START:
        if (b == 1) {
            idle_ones++;
            start_zeros = 0;
        } else {
            start_zeros++;
            if (start_zeros >= 6) idle_ones = 0;
            if (start_zeros >= 2 && idle_ones >= 2) {
                start_zeros = 0;
                stop_ones = 0;
                rx_st = RX_BITS;
                idle_ones = 0;
                rx_byte = 0;
                rx_bit_i = 0;
            }
        }
        break;
    case RX_BITS:
        rx_byte |= (b & 1u) << rx_bit_i;
        rx_bit_i++;
        if (rx_bit_i >= 8) rx_st = RX_STOP;
        break;
    case RX_STOP:
        if (b == 1) {
            stop_ones++;
            if (stop_ones >= 3) {
                fp_push_byte(rx_byte);
                rx_st = RX_WAIT_START;
                bit_cnt = 0;
                start_zeros = 0; idle_ones = 0;
                rx_byte = 0; rx_bit_i = 0; stop_ones = 0;
            }
        } else {
            rx_st = RX_WAIT_START;
            bit_cnt = 0;
            start_zeros = 0; idle_ones = 0;
            rx_byte = 0; rx_bit_i = 0; stop_ones = 0;
        }
        break;
    }
}

static inline void rx_dispatch_decided_bit(uint8_t decided)
{
#if BER_TEST_MODE
    ber_process_bit(decided);
#else
    rx_push_bit(decided);
#endif
}

/* edges realign the window counter, centre-3 majority decides the bit */
static void rx_push_bit_10ms(uint8_t b10)
{
    uint8_t edge = (b10 != prev_raw) ? 1u : 0u;
    prev_raw = b10;

    if (edge) {
        if (bit_cnt >= 3) {
            uint8_t decided = center_vote();
            rx_dispatch_decided_bit(decided);
        }
        bit_cnt = 0;
        win_buf[0] = b10;
        bit_cnt = 1;
        synced = 1;
        return;
    }

    if (bit_cnt < WINS_PER_BIT) {
        win_buf[bit_cnt] = b10;
        bit_cnt++;
    }

    if (bit_cnt >= WINS_PER_BIT) {
        uint8_t decided = center_vote();
        rx_dispatch_decided_bit(decided);
        bit_cnt = 0;
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
    /* 921600 baud: prints must fit in one ADC half-buffer period */
    huart2.Init.BaudRate = 921600;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();

#if BER_TEST_MODE
    printf("\r\n\r\n=== RX BOOT - BER TEST MODE ===\r\n");
    printf("MOD=%s\r\n", (MOD_MODE == 1) ? "OOK (20 kHz carrier, adaptive threshold)"
                                         : "BFSK (10/20 kHz)");
    printf("BPS_MODE=%u (HALF_SAMPLES=%u, %u samples/bit @ %u Hz)\r\n",
           (unsigned)BPS_MODE, (unsigned)HALF_SAMPLES,
           (unsigned)(HALF_SAMPLES * 5u), (unsigned)FS_ADC);
    printf("Expect: 64-bit preamble + Barker13 (1F35h, gated by %u preamble bits = 0x%02X) "
           "+ %u-bit PRBS-9\r\n",
           (unsigned)BER_PREAMBLE_CHECK_BITS, (unsigned)BER_PREAMBLE_EXPECT,
           (unsigned)BER_PRBS_BITS);
    printf("Noise floor band: %.0f Hz; lock threshold: BER<%.2f\r\n",
           (double)F_NOISE, (double)BER_VALID_THRESHOLD);
    if (TEST_DURATION_MIN > 0u) {
        printf("Auto-stop: test will halt %u min after the first sync.\r\n",
               (unsigned)TEST_DURATION_MIN);
    } else {
        printf("Auto-stop: DISABLED (run forever).\r\n");
    }
#else
    printf("\r\n\r\n=== RX BOOT - FRAME MODE ===\r\n");
    printf("MOD=%s BPS_MODE=%u\r\n",
           (MOD_MODE == 1) ? "OOK (20 kHz carrier, adaptive threshold)"
                           : "BFSK (10/20 kHz)",
           (unsigned)BPS_MODE);
#endif

    goertzel_init(&g10,    HALF_SAMPLES, (float)FS_ADC, 10000.0f);
    goertzel_init(&g20,    HALF_SAMPLES, (float)FS_ADC, 20000.0f);
    goertzel_init(&g_noise, HALF_SAMPLES, (float)FS_ADC, F_NOISE);

#if POWER_IDLE_TEST
    /* idle bench: nothing started, WFI */
    printf(">>> POWER_IDLE_TEST: ADC/TIM3/DMA OFF - MCU idle (WFI) <<<\r\n");
#else
    HAL_TIM_Base_Start(&htim3);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, N_SAMPLES);
#endif

    fp_init();
    fp_t0 = HAL_GetTick();
    rx_soft_reset();
    last_bit = 1;

    printf("RX: edge-tracking sync, center-3 voting\r\n");
    printf("Starting decoder...\r\n\r\n");

    uint32_t dbg_t0 = HAL_GetTick();
    uint32_t win_cnt = 0;
    tx_async = 1;   /* banner sent, go non-blocking */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        txr_drain();
#if BER_TEST_MODE
        /* auto-stop: final summary, then idle */
        if (TEST_DURATION_MIN > 0u && !test_done && test_start_ms != 0u &&
            (HAL_GetTick() - test_start_ms) >= TEST_DURATION_MS)
        {
            test_done = 1;
            HAL_ADC_Stop_DMA(&hadc1);
            float ber_cum_f  = (ber_cum_total  > 0)
                               ? (float)ber_cum_err  / (float)ber_cum_total  : 0.0f;
            float ber_cond_f = (ber_valid_bits > 0)
                               ? (float)ber_valid_errs / (float)ber_valid_bits : 0.0f;
            float slip_rate  = (ber_sync_count > 0)
                               ? (float)ber_slip_syncs / (float)ber_sync_count : 0.0f;
            printf("\r\n=========================================\r\n");
            printf("=== END OF TEST (duration=%u min) ===\r\n",
                   (unsigned)TEST_DURATION_MIN);
            printf("=========================================\r\n");
            printf("FINAL: syncs=%lu lock=%lu slip=%lu slip_rate=%.4f\r\n",
                   (unsigned long)ber_sync_count,
                   (unsigned long)ber_valid_syncs,
                   (unsigned long)ber_slip_syncs,
                   (double)slip_rate);
            printf("FINAL: bits_cum=%lu err_cum=%lu BER_cum=%.4e\r\n",
                   (unsigned long)ber_cum_total,
                   (unsigned long)ber_cum_err,
                   (double)ber_cum_f);
            printf("FINAL: bits_lock=%lu err_lock=%lu BER_cond=%.4e\r\n",
                   (unsigned long)ber_valid_bits,
                   (unsigned long)ber_valid_errs,
                   (double)ber_cond_f);
            printf("=========================================\r\n");
            printf("=== STOP LOGGING NOW ===\r\n");
            printf("=========================================\r\n");
            txr_flush_blocking();
        }
        if (test_done) {
            HAL_Delay(1000);
            continue;
        }
#endif /* BER_TEST_MODE */

        if (fp.st != FP_SEARCH && (HAL_GetTick() - fp_t0 > FP_TIMEOUT_MS)) {
            printf("FP TIMEOUT\r\n");
            fp_reset();
            rx_soft_reset();
            fp_t0 = HAL_GetTick();
        }

        uint32_t flags;
        __disable_irq();
        flags = adc_half_ready;
        adc_half_ready = 0;
        __enable_irq();

        if (flags == 0u) {
            __WFI();
            continue;
        }

        for (int half = 0; half < 2; half++) {
            if (!(flags & (1u << half))) continue;

            const uint16_t *x = (half == 0) ? &adc_buf[0] : &adc_buf[HALF_SAMPLES];

            float p10     = goertzel_power(&g10,    x, HALF_SAMPLES);
            float p20     = goertzel_power(&g20,    x, HALF_SAMPLES);
            float p_noise = goertzel_power(&g_noise, x, HALF_SAMPLES);

#if MOD_MODE == 1
            uint8_t bit = classify_ook(p20, p_noise);
#else
            uint8_t bit = classify_bfsk(p10, p20);
#endif

            p10_acc     += p10;
            p20_acc     += p20;
            p_noise_acc += p_noise;
            p_count++;

            rx_push_bit_10ms(bit);
            win_cnt++;
        }

        if (HAL_GetTick() - dbg_t0 >= 5000u) {
            uint32_t t_ms = HAL_GetTick();
            dbg_t0 = t_ms;

            float p10_avg     = (p_count > 0) ? (p10_acc     / (float)p_count) : 0.0f;
            float p20_avg     = (p_count > 0) ? (p20_acc     / (float)p_count) : 0.0f;
            float p_noise_avg = (p_count > 0) ? (p_noise_acc / (float)p_count) : 0.0f;
            float p_sig_max   = (p10_avg > p20_avg) ? p10_avg : p20_avg;
            float p_sig_min   = (p10_avg < p20_avg) ? p10_avg : p20_avg;
            float snr_tone_db = (p_sig_min > 0.0f) ?
                                10.0f * log10f(p_sig_max / p_sig_min) : 0.0f;
            float snr_real_db = (p_sig_max > 0.0f && p_noise_avg > 0.0f) ?
                                10.0f * log10f(p_sig_max / p_noise_avg) : 0.0f;
            p10_acc = 0.0f; p20_acc = 0.0f; p_noise_acc = 0.0f; p_count = 0;

#if BER_TEST_MODE
            float ber_cum  = (ber_cum_total  > 0)
                             ? (float)ber_cum_err  / (float)ber_cum_total  : 0.0f;
            float ber_cond = (ber_valid_bits > 0)
                             ? (float)ber_valid_errs / (float)ber_valid_bits : 0.0f;
            /* CSV,<t_ms>,<win_cnt>,<syncs>,<locks>,<slips>,<bits_cum>,
             * <errs_cum>,<BER_cum>,<BER_cond>,<p10>,<p20>,<p_noise>,
             * <SNR_real_dB>,<SNR_tone_dB> */
            printf("CSV,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%.6e,%.6e,%.1f,%.1f,%.1f,%.2f,%.2f\r\n",
                   (unsigned long)t_ms,
                   (unsigned long)win_cnt,
                   (unsigned long)ber_sync_count,
                   (unsigned long)ber_valid_syncs,
                   (unsigned long)ber_slip_syncs,
                   (unsigned long)ber_cum_total,
                   (unsigned long)ber_cum_err,
                   (double)ber_cum, (double)ber_cond,
                   (double)p10_avg, (double)p20_avg, (double)p_noise_avg,
                   (double)snr_real_db, (double)snr_tone_db);
#else
            printf("RX: wins=%lu ok=%lu fail=%lu gaps=%lu bit=%u st=%u sync=%u\r\n",
                   (unsigned long)win_cnt,
                   (unsigned long)stat_ok,
                   (unsigned long)stat_crc_fail,
                   (unsigned long)stat_gaps,
                   (unsigned)last_bit,
                   (unsigned)fp.st,
                   (unsigned)synced);
            printf("    p10_avg=%.0f p20_avg=%.0f p_noise_avg=%.0f "
                   "SNR_real=%.1f dB SNR_tone=%.1f dB\r\n",
                   (double)p10_avg, (double)p20_avg, (double)p_noise_avg,
                   (double)snr_real_db, (double)snr_tone_db);
#endif
        }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 399;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) adc_half_ready |= 1;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) adc_half_ready |= 2;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
