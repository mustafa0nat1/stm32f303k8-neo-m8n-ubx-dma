/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM32F303K8 + NEO-M8N
  *                   UBX + DMA + Ring Buffer + Auto Recovery
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gps_ubx.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GPS_STATS_PERIOD_MS      5000U
#define MON_VER_TIMEOUT_MS       3000U
#define ACK_TIMEOUT_MS           2000U
#define GPS_RX_TIMEOUT_MS        3000U
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
GPS_UBX_t gps;
GPS_UBX_NAV_PVT_t pvt;
GPS_UBX_MON_VER_t version;

uint32_t last_stats_time = 0U;

volatile uint8_t gps_uart_error_pending = 0U;
volatile uint32_t gps_uart_error_code = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
static void GPS_Process(void);
static void GPS_WaitForMONVER(uint32_t timeout_ms);
static void GPS_WaitForACK(uint8_t expected_class,
                           uint8_t expected_id,
                           uint32_t timeout_ms);
static void GPS_CheckRecovery(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void GPS_Process(void)
{
    GPS_UBX_Process(&gps);
}

static void GPS_WaitForMONVER(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < timeout_ms)
    {
        GPS_Process();

        if (GPS_UBX_GetMONVER(&gps, &version))
        {
            GPS_UBX_DebugPrint(
                &gps,
                "MON-VER CEVABI GELDI -> UBX HABERLESME OK\r\n"
            );

            GPS_UBX_PrintMONVER(
                &gps,
                &version
            );

            return;
        }
    }

    GPS_UBX_DebugPrint(
        &gps,
        "MON-VER CEVABI GELMEDI.\r\n"
    );
}

static void GPS_WaitForACK(uint8_t expected_class,
                           uint8_t expected_id,
                           uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    char message[100];

    while ((HAL_GetTick() - start) < timeout_ms)
    {
        GPS_Process();

        if (gps.ack_status != 0)
        {
            if ((gps.ack_class == expected_class) &&
                (gps.ack_id == expected_id))
            {
                if (gps.ack_status > 0)
                {
                    snprintf(
                        message,
                        sizeof(message),
                        "ACK-ACK | CLASS=0x%02X ID=0x%02X\r\n",
                        gps.ack_class,
                        gps.ack_id
                    );
                }
                else
                {
                    snprintf(
                        message,
                        sizeof(message),
                        "ACK-NAK | CLASS=0x%02X ID=0x%02X\r\n",
                        gps.ack_class,
                        gps.ack_id
                    );
                }

                GPS_UBX_DebugPrint(&gps, message);

                gps.ack_status = 0;
                return;
            }

            gps.ack_status = 0;
        }
    }

    GPS_UBX_DebugPrint(
        &gps,
        "ACK bekleme suresi doldu.\r\n"
    );
}

static void GPS_CheckRecovery(void)
{
    uint8_t need_recovery = 0U;

    if (gps_uart_error_pending != 0U)
    {
        need_recovery = 1U;
    }

    if ((HAL_GetTick() - gps.last_rx_tick) >
        GPS_RX_TIMEOUT_MS)
    {
        need_recovery = 1U;
    }

    if (need_recovery == 0U)
    {
        return;
    }

    {
        char message[120];

        snprintf(
            message,
            sizeof(message),
            "\r\nGPS RX DURDU -> DMA yeniden baslatiliyor... UART_ERR=0x%08lX\r\n",
            (unsigned long)gps_uart_error_code
        );

        GPS_UBX_DebugPrint(
            &gps,
            message
        );
    }

    gps_uart_error_pending = 0U;
    gps_uart_error_code = 0U;

    if (GPS_UBX_RestartDMA(&gps) == HAL_OK)
    {
        GPS_UBX_DebugPrint(
            &gps,
            "GPS DMA RECOVERY OK\r\n"
        );

        /*
         * Kablo geri geldiyse M8N'in tekrar NAV-PVT
         * yayınladığından emin olmak için komutu yeniliyoruz.
         */
        HAL_Delay(100U);

        GPS_UBX_EnableNAVPVT(
            &gps,
            1U
        );
    }
    else
    {
        GPS_UBX_DebugPrint(
            &gps,
            "GPS DMA RECOVERY FAILED\r\n"
        );

        /*
         * Bir sonraki recovery denemesinin hemen spam
         * olmaması için zamanı güncelliyoruz.
         */
        gps.last_rx_tick =
            HAL_GetTick();
    }
}

/* USER CODE END 0 */

int main(void)
{
    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();

    /* USER CODE BEGIN 2 */

    GPS_UBX_Init(
        &gps,
        &huart1,
        &huart2
    );

    /*
     * GPS UART açılır açılmaz DMA başlat.
     */
    if (GPS_UBX_StartDMA(&gps) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_Delay(20U);
    GPS_Process();

    GPS_UBX_DebugPrint(
        &gps,
        "\r\n"
        "============================================\r\n"
        " STM32F303K8 + NEO-M8N\r\n"
        " UBX + DMA + RING BUFFER + FLAG\r\n"
        " GPS     : USART1 / 38400 baud\r\n"
        " TERMITE : USART2 / 115200 baud\r\n"
        "============================================\r\n"
    );

    GPS_UBX_DebugPrint(
        &gps,
        "\r\n[1] GPS RX DMA BASLADI.\r\n"
    );

    GPS_UBX_DebugPrint(
        &gps,
        "\r\n[2] UBX-MON-VER sorgulaniyor...\r\n"
    );

    if (GPS_UBX_RequestMONVER(&gps) == HAL_OK)
    {
        GPS_WaitForMONVER(
            MON_VER_TIMEOUT_MS
        );
    }
    else
    {
        GPS_UBX_DebugPrint(
            &gps,
            "MON-VER komutu gonderilemedi.\r\n"
        );
    }

    GPS_UBX_DebugPrint(
        &gps,
        "\r\n[3] NAV-PVT surekli yayin aciliyor...\r\n"
    );

    if (GPS_UBX_EnableNAVPVT(
            &gps,
            1U) == HAL_OK)
    {
        GPS_UBX_DebugPrint(
            &gps,
            "UBX-CFG-MSG gonderildi.\r\n"
        );

        GPS_WaitForACK(
            UBX_CLASS_CFG,
            UBX_ID_CFG_MSG,
            ACK_TIMEOUT_MS
        );
    }
    else
    {
        GPS_UBX_DebugPrint(
            &gps,
            "NAV-PVT enable komutu gonderilemedi.\r\n"
        );
    }

    GPS_UBX_DebugPrint(
        &gps,
        "\r\n"
        "FLAG tabanli DMA aktif.\r\n"
        "NAV-PVT mesajlari bekleniyor...\r\n\r\n"
    );

    last_stats_time = HAL_GetTick();

    /* USER CODE END 2 */

    while (1)
    {
        GPS_Process();

        GPS_CheckRecovery();

        if (GPS_UBX_GetNAVPVT(
                &gps,
                &pvt))
        {
            GPS_UBX_PrintNAVPVT(
                &gps,
                &pvt
            );
        }

        if (gps.ack_status != 0)
        {
            char message[100];

            if (gps.ack_status > 0)
            {
                snprintf(
                    message,
                    sizeof(message),
                    "ACK-ACK | CLASS=0x%02X ID=0x%02X\r\n",
                    gps.ack_class,
                    gps.ack_id
                );
            }
            else
            {
                snprintf(
                    message,
                    sizeof(message),
                    "ACK-NAK | CLASS=0x%02X ID=0x%02X\r\n",
                    gps.ack_class,
                    gps.ack_id
                );
            }

            GPS_UBX_DebugPrint(
                &gps,
                message
            );

            gps.ack_status = 0;
        }

        if ((HAL_GetTick() - last_stats_time) >=
            GPS_STATS_PERIOD_MS)
        {
            last_stats_time =
                HAL_GetTick();

            GPS_UBX_PrintStats(
                &gps
            );
        }
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    RCC_OscInitStruct.OscillatorType =
        RCC_OSCILLATORTYPE_HSI;

    RCC_OscInitStruct.HSIState =
        RCC_HSI_ON;

    RCC_OscInitStruct.HSICalibrationValue =
        RCC_HSICALIBRATION_DEFAULT;

    RCC_OscInitStruct.PLL.PLLState =
        RCC_PLL_NONE;

    if (HAL_RCC_OscConfig(
            &RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1 |
        RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource =
        RCC_SYSCLKSOURCE_HSI;

    RCC_ClkInitStruct.AHBCLKDivider =
        RCC_SYSCLK_DIV1;

    RCC_ClkInitStruct.APB1CLKDivider =
        RCC_HCLK_DIV1;

    RCC_ClkInitStruct.APB2CLKDivider =
        RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(
            &RCC_ClkInitStruct,
            FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }

    PeriphClkInit.PeriphClockSelection =
        RCC_PERIPHCLK_USART1;

    PeriphClkInit.Usart1ClockSelection =
        RCC_USART1CLKSOURCE_PCLK1;

    if (HAL_RCCEx_PeriphCLKConfig(
            &PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;

    huart1.Init.BaudRate =
        38400;

    huart1.Init.WordLength =
        UART_WORDLENGTH_8B;

    huart1.Init.StopBits =
        UART_STOPBITS_1;

    huart1.Init.Parity =
        UART_PARITY_NONE;

    huart1.Init.Mode =
        UART_MODE_TX_RX;

    huart1.Init.HwFlowCtl =
        UART_HWCONTROL_NONE;

    huart1.Init.OverSampling =
        UART_OVERSAMPLING_16;

    huart1.Init.OneBitSampling =
        UART_ONE_BIT_SAMPLE_DISABLE;

    huart1.AdvancedInit.AdvFeatureInit =
        UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;

    huart2.Init.BaudRate =
        115200;

    huart2.Init.WordLength =
        UART_WORDLENGTH_8B;

    huart2.Init.StopBits =
        UART_STOPBITS_1;

    huart2.Init.Parity =
        UART_PARITY_NONE;

    huart2.Init.Mode =
        UART_MODE_TX_RX;

    huart2.Init.HwFlowCtl =
        UART_HWCONTROL_NONE;

    huart2.Init.OverSampling =
        UART_OVERSAMPLING_16;

    huart2.Init.OneBitSampling =
        UART_ONE_BIT_SAMPLE_DISABLE;

    huart2.AdvancedInit.AdvFeatureInit =
        UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_DMA_Init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();

    HAL_NVIC_SetPriority(
        DMA1_Channel5_IRQn,
        0,
        0
    );

    HAL_NVIC_EnableIRQ(
        DMA1_Channel5_IRQn
    );
}

static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxHalfCpltCallback(
    UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        GPS_UBX_DMAHalfCallback(&gps);
    }
}

void HAL_UART_RxCpltCallback(
    UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        GPS_UBX_DMAFullCallback(&gps);
    }
}

void HAL_UART_ErrorCallback(
    UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        gps_uart_error_code =
            HAL_UART_GetError(huart);

        gps_uart_error_pending =
            1U;
    }
}

/* USER CODE END 4 */

void Error_Handler(void)
{
    __disable_irq();

    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT

void assert_failed(
    uint8_t *file,
    uint32_t line)
{
}

#endif
