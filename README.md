# STM32F303K8 + u-blox NEO-M8N UBX GPS

STM32F303K8 Nucleo ile u-blox NEO-M8N-0-10 GNSS modülünden **UBX protokolü** kullanarak veri alma örneği.

Bu sürümde GPS RX hattı **Circular DMA + Half/Full Transfer callback + software ring buffer + flag + UBX state-machine parser** ile çalışır.

## Özellikler

- UBX binary protokolü
- UBX-MON-VER sorgusu
- UBX-NAV-PVT ayrıştırma
- Circular DMA RX
- DMA Half Transfer / Transfer Complete callback
- Software ring buffer
- `rx_data_flag` ile event tabanlı işleme
- UBX checksum doğrulama
- NAV-PVT sürekli yayın
- UART/DMA otomatik recovery
- Termite/seri terminal çıktısı
- DMA sağlık sayaçları

## Donanım

- STM32 NUCLEO-F303K8
- u-blox NEO-M8N-0-10
- USB üzerinden Nucleo Virtual COM Port
- Seri terminal: Termite

## Bağlantılar

| NEO-M8N | STM32F303K8 |
|---|---|
| TX | PA10 / USART1_RX |
| RX | PA9 / USART1_TX |
| GND | GND |
| VCC | Modül kartının besleme gereksinimine uygun besleme |

Termite/debug hattı:

| STM32 | İşlev |
|---|---|
| PA2 | USART2_TX |
| PA3 | USART2_RX |

## UART ayarları

### USART1 - GPS

- Asynchronous
- 38400 baud
- 8 data bits
- No parity
- 1 stop bit
- No flow control
- PA9 = TX
- PA10 = RX

### USART2 - Termite

- 115200 baud
- 8N1
- No handshake

## DMA ayarları

USART1_RX:

- DMA1 Channel 5
- Peripheral to Memory
- Mode: Circular
- Peripheral Increment: Disable
- Memory Increment: Enable
- Peripheral Data Width: Byte
- Memory Data Width: Byte
- Priority: High

NVIC:

- DMA1 Channel5 interrupt: Enabled
- USART1 global interrupt: Enabled

SYS:

- Debug: Serial Wire

## Çalışma mimarisi

```text
NEO-M8N
   |
USART1 RX
   |
Circular DMA - 256 byte
   |
   +--> İlk 128 byte doldu
   |      HAL_UART_RxHalfCpltCallback()
   |              |
   |              v
   |         Ring Buffer
   |              |
   |         rx_data_flag = 1
   |
   +--> İkinci 128 byte doldu
          HAL_UART_RxCpltCallback()
                  |
                  v
             Ring Buffer
                  |
             rx_data_flag = 1

Main loop
   |
GPS_UBX_Process()
   |
flag varsa
   |
Ring Buffer -> UBX Parser -> NAV-PVT / MON-VER / ACK
```

## Kütüphane yapısı

```text
Core/
├── Inc/
│   ├── gps_ubx.h
│   ├── ring_buffer.h
│   └── ubx.h
└── Src/
    ├── gps_ubx.c
    ├── main.c
    ├── ring_buffer.c
    └── ubx.c
```

### `ubx.c / ubx.h`

UBX protokol katmanıdır.

- `0xB5 0x62` senkronizasyonu
- Class / ID / Length / Payload
- CK_A / CK_B checksum
- UBX paket oluşturma
- Byte-by-byte state machine parser
- Little-endian veri okuma yardımcıları

### `gps_ubx.c / gps_ubx.h`

u-blox GPS sürücüsüdür.

- MON-VER
- NAV-PVT
- ACK / NAK
- DMA callback işleme
- Ring buffer'a veri aktarımı
- `rx_data_flag`
- NAV-PVT decode
- Seri terminal çıktısı
- UART/DMA recovery

### `ring_buffer.c / ring_buffer.h`

Circular DMA'dan gelen blokları CPU tarafında güvenli şekilde biriktirmek için kullanılan software ring buffer katmanıdır.

## UBX-NAV-PVT ile alınan temel alanlar

- UTC tarih / saat
- `fixType`
- `gnssFixOK`
- kullanılan uydu sayısı
- latitude
- longitude
- `height`
- `hMSL`
- horizontal accuracy
- vertical accuracy
- velocity North / East / Down
- ground speed
- heading of motion
- speed accuracy
- heading accuracy
- pDOP

## Örnek çıktı

```text
MON-VER CEVABI GELDI -> UBX HABERLESME OK
SW : EXT CORE 3.01
EXT: FWVER=SPG 3.01
EXT: PROTVER=18.00
EXT: MOD=NEO-M8N-0

ACK-ACK | CLASS=0x06 ID=0x01

PVT | 2026-09-22 14:47:10 |
dateOK=1 | timeOK=1 | resolved=1 |
fix=3 | fixOK=1 | SV=7 |
LAT=+39.xxxxxxx |
LON=+32.xxxxxxx |
ALT=+xxx.xxx m |
hAcc=x.xxx m |
Speed=x.xxx m/s |
Heading=xxx.xxxxx deg

[DMA] NDTR=... | RX=... | Frames=... |
CKERR=0 | DROP=0 | RB_OVF=0 |
HT=... | TC=... | FLAG=0 |
UART_ERR=0x00000000 | REC=0
```

Sağlıklı veri hattında özellikle şunları görmek isteriz:

```text
CKERR=0
DROP=0
RB_OVF=0
```

`HT` ve `TC` sayaçlarının sürekli artması DMA callback'lerinin çalıştığını gösterir.

## Gerekli interrupt handler'lar

CubeMX/CubeIDE tarafından `stm32f3xx_it.c` içinde oluşturulmalıdır:

```c
void DMA1_Channel5_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart1_rx);
}

void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}
```

## Heading hakkında

NAV-PVT içindeki heading, manyetik pusula yönü değil **heading of motion / course over ground** bilgisidir.

- 0 / 360° = Kuzey
- 90° = Doğu
- 180° = Güney
- 270° = Batı

Modül hareketsizken heading güvenilir olmayabilir.

## Recovery

UART hattı kopar veya UART error oluşursa sürücü RX-DMA'yı yeniden başlatabilir.

Örnek:

```text
GPS RX DURDU -> DMA yeniden baslatiliyor...
GPS DMA RECOVERY OK
```

`REC` alanı kaç kez recovery yapıldığını gösterir.

## Not

Bu klasör, projede kullanılan GPS/UBX kaynak kodlarını GitHub'a temiz şekilde koymak için hazırlanmıştır. STM32CubeIDE'nin otomatik ürettiği tüm proje/driver dosyalarını içermez. Yeni bir CubeIDE projesinde yukarıdaki IOC ayarlarını yaptıktan sonra bu `Core/Inc` ve `Core/Src` dosyalarını kullanabilirsiniz.
