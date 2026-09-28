#ifndef GPS_UBX_H
#define GPS_UBX_H

#include "stm32f3xx_hal.h"
#include "ubx.h"
#include "ring_buffer.h"

#include <stdint.h>
#include <stdbool.h>

#define GPS_UBX_DMA_BUFFER_SIZE   256U
#define GPS_UBX_DMA_HALF_SIZE     (GPS_UBX_DMA_BUFFER_SIZE / 2U)
#define GPS_UBX_RING_BUFFER_SIZE  1024U
#define GPS_UBX_MAX_EXTENSIONS    8U

typedef struct
{
    uint32_t iTOW;

    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint8_t valid;

    bool valid_date;
    bool valid_time;
    bool fully_resolved;

    uint8_t fix_type;
    uint8_t flags;
    bool fix_ok;
    uint8_t num_sv;

    int32_t longitude_raw;
    int32_t latitude_raw;

    int32_t height_mm;
    int32_t hmsl_mm;

    uint32_t hacc_mm;
    uint32_t vacc_mm;

    int32_t vel_n_mm_s;
    int32_t vel_e_mm_s;
    int32_t vel_d_mm_s;

    int32_t ground_speed_mm_s;
    int32_t heading_raw;

    uint32_t speed_accuracy_mm_s;
    uint32_t heading_accuracy_raw;

    uint16_t pdop_raw;
    int32_t vehicle_heading_raw;
} GPS_UBX_NAV_PVT_t;

typedef struct
{
    char software[31];
    char hardware[11];

    char extensions[GPS_UBX_MAX_EXTENSIONS][31];
    uint8_t extension_count;

    uint16_t payload_length;
} GPS_UBX_MON_VER_t;

typedef struct
{
    UART_HandleTypeDef *gps_uart;
    UART_HandleTypeDef *debug_uart;

    uint8_t dma_buffer[GPS_UBX_DMA_BUFFER_SIZE];
    uint16_t dma_old_position;

    uint8_t ring_memory[GPS_UBX_RING_BUFFER_SIZE];
    RingBuffer_t ring;

    UBX_Parser_t parser;

    GPS_UBX_NAV_PVT_t latest_pvt;
    GPS_UBX_MON_VER_t latest_mon_ver;

    volatile bool pvt_ready;
    volatile bool mon_ver_ready;

    volatile int8_t ack_status;
    uint8_t ack_class;
    uint8_t ack_id;

    uint32_t received_bytes;
    uint32_t dropped_bytes;
    uint32_t parsed_frames;
    uint32_t checksum_errors;

    volatile uint8_t rx_data_flag;
    uint32_t dma_half_events;
    uint32_t dma_full_events;

    volatile uint32_t last_rx_tick;
    volatile uint32_t recovery_count;
} GPS_UBX_t;

void GPS_UBX_Init(GPS_UBX_t *gps,
                  UART_HandleTypeDef *gps_uart,
                  UART_HandleTypeDef *debug_uart);

HAL_StatusTypeDef GPS_UBX_StartDMA(GPS_UBX_t *gps);
HAL_StatusTypeDef GPS_UBX_RestartDMA(GPS_UBX_t *gps);

void GPS_UBX_DMAHalfCallback(GPS_UBX_t *gps);
void GPS_UBX_DMAFullCallback(GPS_UBX_t *gps);
void GPS_UBX_Process(GPS_UBX_t *gps);

HAL_StatusTypeDef GPS_UBX_SendPacket(GPS_UBX_t *gps,
                                     uint8_t msg_class,
                                     uint8_t msg_id,
                                     const uint8_t *payload,
                                     uint16_t payload_length);

HAL_StatusTypeDef GPS_UBX_SendPoll(GPS_UBX_t *gps,
                                   uint8_t msg_class,
                                   uint8_t msg_id);

HAL_StatusTypeDef GPS_UBX_RequestMONVER(GPS_UBX_t *gps);
HAL_StatusTypeDef GPS_UBX_RequestNAVPVT(GPS_UBX_t *gps);

HAL_StatusTypeDef GPS_UBX_EnableNAVPVT(GPS_UBX_t *gps,
                                       uint8_t rate);

bool GPS_UBX_GetNAVPVT(GPS_UBX_t *gps,
                       GPS_UBX_NAV_PVT_t *pvt);

bool GPS_UBX_GetMONVER(GPS_UBX_t *gps,
                       GPS_UBX_MON_VER_t *version);

bool GPS_UBX_DecodeNAVPVT(const UBX_Frame_t *frame,
                          GPS_UBX_NAV_PVT_t *pvt);

bool GPS_UBX_DecodeMONVER(const UBX_Frame_t *frame,
                          GPS_UBX_MON_VER_t *version);

void GPS_UBX_DebugPrint(GPS_UBX_t *gps,
                        const char *text);

void GPS_UBX_PrintNAVPVT(GPS_UBX_t *gps,
                         const GPS_UBX_NAV_PVT_t *pvt);

void GPS_UBX_PrintMONVER(GPS_UBX_t *gps,
                         const GPS_UBX_MON_VER_t *version);

void GPS_UBX_PrintStats(GPS_UBX_t *gps);

double GPS_UBX_LatitudeDeg(const GPS_UBX_NAV_PVT_t *pvt);
double GPS_UBX_LongitudeDeg(const GPS_UBX_NAV_PVT_t *pvt);
double GPS_UBX_AltitudeMSL_M(const GPS_UBX_NAV_PVT_t *pvt);
double GPS_UBX_GroundSpeed_MPS(const GPS_UBX_NAV_PVT_t *pvt);
double GPS_UBX_HeadingDeg(const GPS_UBX_NAV_PVT_t *pvt);
double GPS_UBX_HAcc_M(const GPS_UBX_NAV_PVT_t *pvt);

#endif
