# STM32CubeMX / STM32CubeIDE Ayarları

## SYS

- Debug: Serial Wire

## USART1

- Mode: Asynchronous
- Baud Rate: 38400
- Word Length: 8 Bits
- Parity: None
- Stop Bits: 1
- Hardware Flow Control: Disable
- PA9: USART1_TX
- PA10: USART1_RX

## USART1 RX DMA

- DMA Request: USART1_RX
- Channel: DMA1 Channel 5
- Direction: Peripheral to Memory
- Mode: Circular
- Peripheral Increment: Disable
- Memory Increment: Enable
- Peripheral Data Width: Byte
- Memory Data Width: Byte
- Priority: High

## NVIC

- DMA1 Channel5 interrupt: Enabled
- USART1 global interrupt: Enabled

## USART2

- Mode: Asynchronous
- Baud Rate: 115200
- 8N1
- PA2: USART2_TX
- PA3: USART2_RX

## Kontrol

`stm32f3xx_hal_msp.c` içinde aşağıdaki satırların oluştuğunu doğrulayın:

```c
hdma_usart1_rx.Instance = DMA1_Channel5;
hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
hdma_usart1_rx.Init.Priority = DMA_PRIORITY_HIGH;

HAL_DMA_Init(&hdma_usart1_rx);
__HAL_LINKDMA(huart, hdmarx, hdma_usart1_rx);
```
