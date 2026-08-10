/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : IBC transmitter (BFSK/OOK)
 ******************************************************************************
 *
 * BER_TEST_MODE: 1 = BER stream (PRBS-9 + Barker-13), 0 = packet mode.
 * BPS_MODE 20/40/100, MOD_MODE 0 = BFSK / 1 = OOK. Same values on RX.
 *
 * NUCLEO-L476RG @ 80 MHz. DAC1 OUT1 (PA4) -> signal electrode, GND ->
 * return electrode. TIM7 200 kHz DAC trigger, TIM6 bit timer, USART2 log.
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
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 1 = BER stream, 0 = packet mode */
#define BER_TEST_MODE   0

/* 1 = idle power bench (signal chain off, WFI) */
#ifndef POWER_IDLE_TEST
#define POWER_IDLE_TEST 0
#endif

#define PKT_TEST_COUNT     100u   /* frames per run */
#define PKT_GLUCOSE_MGDL   120u   /* payload, mg/dL */

/* bit rate: 20, 40 or 100 bps (same on RX) */
#ifndef BPS_MODE
#define BPS_MODE 20
#endif

/* 0 = BFSK (10/20 kHz), 1 = OOK (20 kHz, off = mid-scale) */
#ifndef MOD_MODE
#define MOD_MODE 1
#endif
#if MOD_MODE != 0 && MOD_MODE != 1
#error "MOD_MODE must be 0 (BFSK) or 1 (OOK)"
#endif

#if BPS_MODE == 20
  #define TIM6_PSC      9999u
  #define TIM6_ARR      399u    /* 10000*400 / 80MHz = 50.00 ms/bit -> 20 bps */
  #define BIT_PERIOD_US 50000u
#elif BPS_MODE == 40
  #define TIM6_PSC      9999u
  #define TIM6_ARR      199u    /* 10000*200 / 80MHz = 25.00 ms/bit -> 40 bps */
  #define BIT_PERIOD_US 25000u
#elif BPS_MODE == 100
  #define TIM6_PSC      9999u
  #define TIM6_ARR      79u     /* 10000*80 / 80MHz = 10.00 ms/bit -> 100 bps */
  #define BIT_PERIOD_US 10000u
#else
  #error "Unsupported BPS_MODE: choose 20, 40 or 100"
#endif

#define FS_DAC          200000u
#define F0              10000u
#define F1              20000u

#define SINE10_SAMPLES  (FS_DAC / F0)
#define SINE20_SAMPLES  (FS_DAC / F1)

static uint16_t sine10k[SINE10_SAMPLES];
static uint16_t sine20k[SINE20_SAMPLES];

/* 40 = integer periods of both tones -> phase-continuous swaps */
#define DMA_BUF_LEN     40u

static uint16_t dac_buf_10k[DMA_BUF_LEN];
static uint16_t dac_buf_20k[DMA_BUF_LEN];
#if MOD_MODE == 1
static uint16_t dac_buf_flat[DMA_BUF_LEN];  /* OOK carrier off */
#endif
static uint16_t dac_live[DMA_BUF_LEN];      /* DAC DMA source */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac_ch1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static volatile uint8_t tx_bit_level         = 1;
static volatile uint8_t tx_bit_level_applied = 1;

typedef enum {
    TX_IDLE,
    TX_IDLE1, TX_IDLE2, TX_IDLE3, TX_IDLE4,
    TX_START1, TX_START2,
    TX_DATA,
    TX_STOP1, TX_STOP2,
    TX_GAP1, TX_GAP2, TX_GAP3, TX_GAP4, TX_GAP5, TX_GAP6
} tx_state_t;

static volatile tx_state_t tx_st    = TX_IDLE;
static volatile uint8_t    tx_active  = 0;
static volatile uint8_t    tx_cur_byte = 0;
static volatile uint8_t    tx_bit_idx  = 0;

#define TXQ_SZ 128
static volatile uint8_t txq[TXQ_SZ];
static volatile uint8_t txq_head = 0, txq_tail = 0;

static int txq_push(uint8_t b)
{
    uint8_t n = (uint8_t)((txq_head + 1) % TXQ_SZ);
    if (n == txq_tail) return -1;
    txq[txq_head] = b;
    txq_head = n;
    return 0;
}

static int txq_pop(uint8_t *b)
{
    if (txq_head == txq_tail) return -1;
    *b = txq[txq_tail];
    txq_tail = (uint8_t)((txq_tail + 1) % TXQ_SZ);
    return 0;
}

static volatile uint32_t dbg_tick_count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int __io_putchar(int ch)
{
    uint8_t c = (uint8_t)ch;
    HAL_UART_Transmit(&huart2, &c, 1, 10);
    return ch;
}

/* frame: 16x 0x55, sync 0xD3 0x91, LEN, SEQ, payload, CRC-16/CCITT-FALSE */
#define PREAMBLE_BYTE   0x55u
#define PREAMBLE_LEN    16u
#define SYNC1_BYTE      0xD3u
#define SYNC2_BYTE      0x91u

static uint16_t crc16_ccitt_false_update(uint16_t crc, uint8_t data)
{
    crc ^= (uint16_t)data << 8;
    for (int i = 0; i < 8; i++) {
        if (crc & 0x8000) crc = (uint16_t)((crc << 1) ^ 0x1021);
        else              crc = (uint16_t)(crc << 1);
    }
    return crc;
}

static uint16_t crc16_ccitt_false(const uint8_t *buf, uint32_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint32_t i = 0; i < len; i++)
        crc = crc16_ccitt_false_update(crc, buf[i]);
    return crc;
}

static uint8_t tx_seq = 0;

static void tx_send_ibc_frame(uint8_t seq, const uint8_t *payload, uint8_t len)
{
    for (uint8_t i = 0; i < PREAMBLE_LEN; i++) txq_push(PREAMBLE_BYTE);

    txq_push(SYNC1_BYTE);
    txq_push(SYNC2_BYTE);
    txq_push(len);
    txq_push(seq);

    for (uint8_t i = 0; i < len; i++) txq_push(payload[i]);

    /* CRC over LEN..payload */
    uint8_t tmp[2 + 255];
    tmp[0] = len;
    tmp[1] = seq;
    for (uint8_t i = 0; i < len; i++) tmp[2 + i] = payload[i];
    uint16_t crc = crc16_ccitt_false(tmp, (uint32_t)(2u + len));
    txq_push((uint8_t)(crc & 0xFFu));
    txq_push((uint8_t)((crc >> 8) & 0xFFu));

    if (!tx_active) {
        tx_active = 1;
        tx_st = TX_IDLE1;
    }
}

static void tx_apply_bit(void)
{
    if (tx_bit_level == tx_bit_level_applied) return;
    tx_bit_level_applied = tx_bit_level;

    /* in-place swap, DMA never stops (no underrun) */
#if MOD_MODE == 1
    const uint16_t *src = tx_bit_level ? dac_buf_20k : dac_buf_flat;
#else
    const uint16_t *src = tx_bit_level ? dac_buf_20k : dac_buf_10k;
#endif
    for (uint32_t i = 0; i < DMA_BUF_LEN; i++) dac_live[i] = src[i];
}

static inline void tx_set_bit(uint8_t b)
{
    tx_bit_level = b;
    tx_apply_bit();
}

#if BER_TEST_MODE
/* stream: 64-bit preamble + Barker-13 + 511-bit PRBS-9 (x^9+x^5+1) */

static uint16_t prbs9_state = 0x1FFu;
static uint8_t prbs9_next_bit(void)
{
    uint8_t out = (uint8_t)(((prbs9_state >> 8) ^ (prbs9_state >> 4)) & 1u);
    prbs9_state = (uint16_t)(((prbs9_state << 1) | out) & 0x1FFu);
    return out;
}

#define BER_PREAMBLE_BITS  64u
#define BER_BARKER_LEN     13u
#define BER_PRBS_BITS      511u
static const uint8_t barker13[BER_BARKER_LEN] = { 1,1,1,1,1,0,0,1,1,0,1,0,1 };

typedef enum { BER_PHASE_PREAMBLE = 0, BER_PHASE_BARKER, BER_PHASE_PRBS } ber_phase_t;
static volatile ber_phase_t ber_phase     = BER_PHASE_PREAMBLE;
static volatile uint32_t    ber_phase_cnt = 0;

static uint8_t ber_next_bit(void)
{
    uint8_t b = 0;
    switch (ber_phase) {
    case BER_PHASE_PREAMBLE:
        b = (uint8_t)(ber_phase_cnt & 1u);
        if (++ber_phase_cnt >= BER_PREAMBLE_BITS) {
            ber_phase = BER_PHASE_BARKER;
            ber_phase_cnt = 0;
        }
        break;
    case BER_PHASE_BARKER:
        b = barker13[ber_phase_cnt];
        if (++ber_phase_cnt >= BER_BARKER_LEN) {
            ber_phase = BER_PHASE_PRBS;
            ber_phase_cnt = 0;
            prbs9_state = 0x1FFu;   /* re-seed */
        }
        break;
    case BER_PHASE_PRBS:
    default:
        b = prbs9_next_bit();
        if (++ber_phase_cnt >= BER_PRBS_BITS) {
            ber_phase = BER_PHASE_PREAMBLE;
            ber_phase_cnt = 0;
        }
        break;
    }
    return b;
}
#endif /* BER_TEST_MODE */

/* one bit per TIM6 period */
static void tx_tick(void)
{
    dbg_tick_count++;
#if BER_TEST_MODE
    tx_set_bit(ber_next_bit());
    return;
#else
    switch (tx_st)
    {
    case TX_IDLE:
        tx_set_bit(1);
        tx_active = 0;
        break;
    case TX_IDLE1: tx_set_bit(1); tx_st = TX_IDLE2; break;
    case TX_IDLE2: tx_set_bit(1); tx_st = TX_IDLE3; break;
    case TX_IDLE3: tx_set_bit(1); tx_st = TX_IDLE4; break;
    case TX_IDLE4: tx_set_bit(1); tx_st = TX_START1; break;
    case TX_START1:
        tx_set_bit(0);
        tx_st = TX_START2;
        break;
    case TX_START2:
        tx_set_bit(0);
        if (txq_pop((uint8_t *)&tx_cur_byte) == 0) {
            tx_bit_idx = 0;
            tx_st = TX_DATA;
        } else {
            tx_st = TX_IDLE;
        }
        break;
    case TX_DATA: {
        uint8_t b = (uint8_t)((tx_cur_byte >> tx_bit_idx) & 1u);
        tx_set_bit(b);
        tx_bit_idx++;
        if (tx_bit_idx >= 8) tx_st = TX_STOP1;
    } break;
    case TX_STOP1: tx_set_bit(1); tx_st = TX_STOP2; break;
    case TX_STOP2: tx_set_bit(1); tx_st = TX_GAP1;  break;
    case TX_GAP1:  tx_set_bit(1); tx_st = TX_GAP2;  break;
    case TX_GAP2:  tx_set_bit(1); tx_st = TX_GAP3;  break;
    case TX_GAP3:  tx_set_bit(1); tx_st = TX_GAP4;  break;
    case TX_GAP4:  tx_set_bit(1); tx_st = TX_GAP5;  break;
    case TX_GAP5:  tx_set_bit(1); tx_st = TX_GAP6;  break;
    case TX_GAP6:  tx_set_bit(1); tx_st = TX_START1; break;
    }
#endif /* !BER_TEST_MODE */
}

static void build_sine_table(uint16_t *buf, uint32_t n_samples, uint16_t amplitude)
{
    const float mid = 2048.0f;
    for (uint32_t i = 0; i < n_samples; i++) {
        float s = sinf(2.0f * 3.14159265f * (float)i / (float)n_samples);
        float v = mid + (float)amplitude * s;
        if (v < 0.0f)    v = 0.0f;
        if (v > 4095.0f) v = 4095.0f;
        buf[i] = (uint16_t)v;
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
  MX_USART2_UART_Init();
  MX_DAC1_Init();
  MX_TIM7_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
    printf("\r\n\r\n=== TX Node - GC-IBC BFSK ===\r\n");
    printf("Clock: %s @ 80 MHz\r\n",
           __HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) ? "MSI+LSE (crystal, stable)"
                                               : "MSI free-run (LSE FAILED - clock drifts, DO NOT record)");
#if MOD_MODE == 1
    printf("OOK: 20 kHz carrier, on=bit1 off=bit0 (DAC held at 2048), %u bps (BPS_MODE=%u)\r\n",
           (unsigned)BPS_MODE, (unsigned)BPS_MODE);
#else
    printf("BFSK: 10 kHz=bit0, 20 kHz=bit1, %u bps (BPS_MODE=%u)\r\n",
           (unsigned)BPS_MODE, (unsigned)BPS_MODE);
#endif
    printf("Frame: 16x preamble + SYNC + LEN + SEQ + PAYLOAD + CRC16\r\n\r\n");

    build_sine_table(sine10k, SINE10_SAMPLES, 700);
    build_sine_table(sine20k, SINE20_SAMPLES, 700);
    for (uint32_t i = 0; i < DMA_BUF_LEN; i++) {
        dac_buf_10k[i] = sine10k[i % SINE10_SAMPLES];
        dac_buf_20k[i] = sine20k[i % SINE20_SAMPLES];
#if MOD_MODE == 1
        dac_buf_flat[i] = 2048u;            /* carrier off */
        dac_live[i]     = dac_buf_flat[i];
#else
        dac_live[i]    = dac_buf_10k[i];
#endif
    }

#if POWER_IDLE_TEST
    /* idle bench: nothing started, WFI */
    printf(">>> POWER_IDLE_TEST: DAC/DMA/TIM6/TIM7 OFF - MCU idle (WFI) <<<\r\n");
    while (1) { __WFI(); }
#endif

    /* DAC DMA: circular on dac_live, started once, never stopped */
    tx_bit_level = 0; tx_bit_level_applied = 0;
    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1,
                      (uint32_t *)dac_live, DMA_BUF_LEN, DAC_ALIGN_12B_R);
    __HAL_DMA_DISABLE_IT(&hdma_dac_ch1, DMA_IT_HT | DMA_IT_TC);
    HAL_TIM_Base_Start(&htim7);

    /* bench tone test */
#if MOD_MODE == 1
    printf("Tone test: 1s carrier OFF...\r\n");
    tx_set_bit(0);
    HAL_Delay(1000);
    printf("Tone test: 1s carrier ON (20 kHz)...\r\n");
    tx_set_bit(1);
    HAL_Delay(1000);
#else
    printf("Tone test: 1s @ 10kHz...\r\n");
    tx_set_bit(0);
    HAL_Delay(1000);
    printf("Tone test: 1s @ 20kHz...\r\n");
    tx_set_bit(1);
    HAL_Delay(1000);
#endif

    tx_set_bit(1);
    tx_active = 0; tx_st = TX_IDLE;

    /* re-init TIM6 with the BPS_MODE period (CubeMX default is 20 bps) */
    htim6.Init.Prescaler = TIM6_PSC;
    htim6.Init.Period    = TIM6_ARR;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK) Error_Handler();

    HAL_TIM_Base_Start_IT(&htim6);

#if BER_TEST_MODE
    printf("TX BER MODE: BPS=%u preamble=%u barker=%u PRBS-9=%u bits, cycle=%u bits (%lu us)\r\n",
           (unsigned)BPS_MODE,
           (unsigned)BER_PREAMBLE_BITS, (unsigned)BER_BARKER_LEN, (unsigned)BER_PRBS_BITS,
           (unsigned)(BER_PREAMBLE_BITS + BER_BARKER_LEN + BER_PRBS_BITS),
           (unsigned long)((BER_PREAMBLE_BITS + BER_BARKER_LEN + BER_PRBS_BITS) * (unsigned long)BIT_PERIOD_US));
    uint32_t last_dbg = HAL_GetTick();
    while (1)
    {
        if (HAL_GetTick() - last_dbg >= 5000u) {
            last_dbg = HAL_GetTick();
            printf("TX DBG: ticks=%lu phase=%u cnt=%lu\r\n",
                   (unsigned long)dbg_tick_count,
                   (unsigned)ber_phase, (unsigned long)ber_phase_cnt);
        }
    }
#else
    /* send PKT_TEST_COUNT frames (uint16 LE payload), then idle */
    printf("TX started. Packet test: %u frames, payload=%u mg/dL (uint16 LE).\r\n\r\n",
           (unsigned)PKT_TEST_COUNT, (unsigned)PKT_GLUCOSE_MGDL);

    while (1)
    {
        if (!tx_active && tx_seq < PKT_TEST_COUNT) {
            HAL_Delay(100);
            const uint8_t msg[2] = {
                (uint8_t)(PKT_GLUCOSE_MGDL & 0xFFu),
                (uint8_t)((PKT_GLUCOSE_MGDL >> 8) & 0xFFu)
            };
            tx_send_ibc_frame(tx_seq, msg, sizeof(msg));
            tx_seq++;
            if (tx_seq == PKT_TEST_COUNT)
                printf("TX done: %u frames queued. Idle.\r\n", (unsigned)PKT_TEST_COUNT);
        }
    }
#endif /* BER_TEST_MODE */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  * (high drive + backup-domain reset, LSE fails to start otherwise)
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_BACKUPRESET_FORCE();
  __HAL_RCC_BACKUPRESET_RELEASE();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_HIGH);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* LSE failed: MSI-only fallback, clock drifts - do not record */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      Error_Handler();
    }
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration (MSI locked to LSE) */
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY))
  {
    HAL_RCCEx_EnableMSIPLLMode();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T7_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 9999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 399;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 399;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* TIM6 update ISR: advance the bit-level TX state machine */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) tx_tick();
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
