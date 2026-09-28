#include "gps_ubx.h"

#include <string.h>
#include <stdio.h>

static void GPS_UBX_CopyToRing(GPS_UBX_t *gps,
                               const uint8_t *data,
                               uint16_t length)
{
    uint16_t written;

    written = RingBuffer_Write(
        &gps->ring,
        data,
        length
    );

    gps->received_bytes += written;

    if (written > 0U)
    {
        gps->last_rx_tick = HAL_GetTick();
        gps->rx_data_flag = 1U;
    }

    if (written < length)
    {
        gps->dropped_bytes +=
            (uint32_t)(length - written);
    }
}

static void GPS_UBX_ClearRxState(GPS_UBX_t *gps)
{
    volatile uint32_t dummy;

    if ((gps == NULL) ||
        (gps->gps_uart == NULL))
    {
        return;
    }

    if (__HAL_UART_GET_FLAG(
            gps->gps_uart,
            UART_FLAG_RXNE) != RESET)
    {
        dummy = gps->gps_uart->Instance->RDR;
        (void)dummy;
    }

    __HAL_UART_CLEAR_OREFLAG(gps->gps_uart);
    __HAL_UART_CLEAR_FEFLAG(gps->gps_uart);
    __HAL_UART_CLEAR_NEFLAG(gps->gps_uart);

    gps->gps_uart->ErrorCode =
        HAL_UART_ERROR_NONE;
}

void GPS_UBX_Init(GPS_UBX_t *gps,
                  UART_HandleTypeDef *gps_uart,
                  UART_HandleTypeDef *debug_uart)
{
    if (gps == NULL)
    {
        return;
    }

    memset(gps, 0, sizeof(GPS_UBX_t));

    gps->gps_uart = gps_uart;
    gps->debug_uart = debug_uart;

    RingBuffer_Init(
        &gps->ring,
        gps->ring_memory,
        GPS_UBX_RING_BUFFER_SIZE
    );

    UBX_ParserInit(&gps->parser);

    gps->last_rx_tick = HAL_GetTick();
    gps->recovery_count = 0U;
}

HAL_StatusTypeDef GPS_UBX_StartDMA(GPS_UBX_t *gps)
{
    HAL_StatusTypeDef status;

    if ((gps == NULL) ||
        (gps->gps_uart == NULL) ||
        (gps->gps_uart->hdmarx == NULL))
    {
        return HAL_ERROR;
    }

    RingBuffer_Clear(&gps->ring);
    UBX_ParserInit(&gps->parser);

    gps->rx_data_flag = 0U;

    GPS_UBX_ClearRxState(gps);

    status = HAL_UART_Receive_DMA(
        gps->gps_uart,
        gps->dma_buffer,
        GPS_UBX_DMA_BUFFER_SIZE
    );

    if (status == HAL_OK)
    {
        gps->last_rx_tick = HAL_GetTick();
    }

    return status;
}

HAL_StatusTypeDef GPS_UBX_RestartDMA(GPS_UBX_t *gps)
{
    HAL_StatusTypeDef status;

    if ((gps == NULL) ||
        (gps->gps_uart == NULL) ||
        (gps->gps_uart->hdmarx == NULL))
    {
        return HAL_ERROR;
    }

    HAL_UART_DMAStop(gps->gps_uart);

    GPS_UBX_ClearRxState(gps);

    RingBuffer_Clear(&gps->ring);
    UBX_ParserInit(&gps->parser);

    gps->rx_data_flag = 0U;
    gps->pvt_ready = false;
    gps->mon_ver_ready = false;
    gps->ack_status = 0;

    status = HAL_UART_Receive_DMA(
        gps->gps_uart,
        gps->dma_buffer,
        GPS_UBX_DMA_BUFFER_SIZE
    );

    if (status == HAL_OK)
    {
        gps->last_rx_tick = HAL_GetTick();
        gps->recovery_count++;
    }

    return status;
}

void GPS_UBX_DMAHalfCallback(GPS_UBX_t *gps)
{
    if (gps == NULL)
    {
        return;
    }

    GPS_UBX_CopyToRing(
        gps,
        &gps->dma_buffer[0],
        GPS_UBX_DMA_HALF_SIZE
    );

    gps->dma_half_events++;
}

void GPS_UBX_DMAFullCallback(GPS_UBX_t *gps)
{
    if (gps == NULL)
    {
        return;
    }

    GPS_UBX_CopyToRing(
        gps,
        &gps->dma_buffer[GPS_UBX_DMA_HALF_SIZE],
        GPS_UBX_DMA_HALF_SIZE
    );

    gps->dma_full_events++;
}

HAL_StatusTypeDef GPS_UBX_SendPacket(
    GPS_UBX_t *gps,
    uint8_t msg_class,
    uint8_t msg_id,
    const uint8_t *payload,
    uint16_t payload_length)
{
    uint8_t packet[UBX_MAX_PAYLOAD + 8U];
    uint16_t packet_length;

    if ((gps == NULL) ||
        (gps->gps_uart == NULL) ||
        (payload_length > UBX_MAX_PAYLOAD))
    {
        return HAL_ERROR;
    }

    packet_length =
        UBX_BuildPacket(
            msg_class,
            msg_id,
            payload,
            payload_length,
            packet,
            sizeof(packet)
        );

    if (packet_length == 0U)
    {
        return HAL_ERROR;
    }

    return HAL_UART_Transmit(
        gps->gps_uart,
        packet,
        packet_length,
        200U
    );
}

HAL_StatusTypeDef GPS_UBX_SendPoll(
    GPS_UBX_t *gps,
    uint8_t msg_class,
    uint8_t msg_id)
{
    return GPS_UBX_SendPacket(
        gps,
        msg_class,
        msg_id,
        NULL,
        0U
    );
}

HAL_StatusTypeDef GPS_UBX_RequestMONVER(
    GPS_UBX_t *gps)
{
    if (gps == NULL)
    {
        return HAL_ERROR;
    }

    gps->mon_ver_ready = false;

    return GPS_UBX_SendPoll(
        gps,
        UBX_CLASS_MON,
        UBX_ID_MON_VER
    );
}

HAL_StatusTypeDef GPS_UBX_RequestNAVPVT(
    GPS_UBX_t *gps)
{
    if (gps == NULL)
    {
        return HAL_ERROR;
    }

    gps->pvt_ready = false;

    return GPS_UBX_SendPoll(
        gps,
        UBX_CLASS_NAV,
        UBX_ID_NAV_PVT
    );
}

HAL_StatusTypeDef GPS_UBX_EnableNAVPVT(
    GPS_UBX_t *gps,
    uint8_t rate)
{
    uint8_t payload[3];

    if (gps == NULL)
    {
        return HAL_ERROR;
    }

    payload[0] = UBX_CLASS_NAV;
    payload[1] = UBX_ID_NAV_PVT;
    payload[2] = rate;

    gps->ack_status = 0;

    return GPS_UBX_SendPacket(
        gps,
        UBX_CLASS_CFG,
        UBX_ID_CFG_MSG,
        payload,
        sizeof(payload)
    );
}

bool GPS_UBX_DecodeNAVPVT(
    const UBX_Frame_t *frame,
    GPS_UBX_NAV_PVT_t *pvt)
{
    const uint8_t *payload;

    if ((frame == NULL) ||
        (pvt == NULL) ||
        (frame->msg_class != UBX_CLASS_NAV) ||
        (frame->msg_id != UBX_ID_NAV_PVT) ||
        (frame->length != 92U))
    {
        return false;
    }

    payload = frame->payload;

    memset(
        pvt,
        0,
        sizeof(GPS_UBX_NAV_PVT_t)
    );

    pvt->iTOW = UBX_ReadU32(&payload[0]);

    pvt->year = UBX_ReadU16(&payload[4]);
    pvt->month = payload[6];
    pvt->day = payload[7];
    pvt->hour = payload[8];
    pvt->minute = payload[9];
    pvt->second = payload[10];
    pvt->valid = payload[11];

    pvt->valid_date =
        ((payload[11] & 0x01U) != 0U);

    pvt->valid_time =
        ((payload[11] & 0x02U) != 0U);

    pvt->fully_resolved =
        ((payload[11] & 0x04U) != 0U);

    pvt->fix_type = payload[20];
    pvt->flags = payload[21];

    pvt->fix_ok =
        ((payload[21] & 0x01U) != 0U);

    pvt->num_sv = payload[23];

    pvt->longitude_raw =
        UBX_ReadI32(&payload[24]);

    pvt->latitude_raw =
        UBX_ReadI32(&payload[28]);

    pvt->height_mm =
        UBX_ReadI32(&payload[32]);

    pvt->hmsl_mm =
        UBX_ReadI32(&payload[36]);

    pvt->hacc_mm =
        UBX_ReadU32(&payload[40]);

    pvt->vacc_mm =
        UBX_ReadU32(&payload[44]);

    pvt->vel_n_mm_s =
        UBX_ReadI32(&payload[48]);

    pvt->vel_e_mm_s =
        UBX_ReadI32(&payload[52]);

    pvt->vel_d_mm_s =
        UBX_ReadI32(&payload[56]);

    pvt->ground_speed_mm_s =
        UBX_ReadI32(&payload[60]);

    pvt->heading_raw =
        UBX_ReadI32(&payload[64]);

    pvt->speed_accuracy_mm_s =
        UBX_ReadU32(&payload[68]);

    pvt->heading_accuracy_raw =
        UBX_ReadU32(&payload[72]);

    pvt->pdop_raw =
        UBX_ReadU16(&payload[76]);

    pvt->vehicle_heading_raw =
        UBX_ReadI32(&payload[84]);

    return true;
}

bool GPS_UBX_DecodeMONVER(
    const UBX_Frame_t *frame,
    GPS_UBX_MON_VER_t *version)
{
    uint16_t offset;

    if ((frame == NULL) ||
        (version == NULL) ||
        (frame->msg_class != UBX_CLASS_MON) ||
        (frame->msg_id != UBX_ID_MON_VER) ||
        (frame->length < 40U))
    {
        return false;
    }

    memset(
        version,
        0,
        sizeof(GPS_UBX_MON_VER_t)
    );

    memcpy(
        version->software,
        &frame->payload[0],
        30U
    );

    version->software[30] = '\0';

    memcpy(
        version->hardware,
        &frame->payload[30],
        10U
    );

    version->hardware[10] = '\0';

    version->payload_length =
        frame->length;

    offset = 40U;

    while (((offset + 30U) <= frame->length) &&
           (version->extension_count <
            GPS_UBX_MAX_EXTENSIONS))
    {
        memcpy(
            version->extensions[
                version->extension_count
            ],
            &frame->payload[offset],
            30U
        );

        version->extensions[
            version->extension_count
        ][30] = '\0';

        version->extension_count++;

        offset += 30U;
    }

    return true;
}

void GPS_UBX_Process(
    GPS_UBX_t *gps)
{
    uint8_t byte;

    if (gps == NULL)
    {
        return;
    }

    if (gps->rx_data_flag == 0U)
    {
        return;
    }

    /*
     * Once flag sifirlanir. Isleme sirasinda yeni DMA callback gelirse
     * callback flag'i tekrar 1 yapar.
     */
    gps->rx_data_flag = 0U;

    while (RingBuffer_Pop(
            &gps->ring,
            &byte))
    {
        UBX_ParseResult_t result;

        result = UBX_ParseByte(
            &gps->parser,
            byte
        );

        if (result ==
            UBX_PARSE_FRAME_READY)
        {
            const UBX_Frame_t *frame;

            frame = UBX_GetFrame(
                &gps->parser
            );

            gps->parsed_frames++;

            if ((frame->msg_class ==
                 UBX_CLASS_NAV) &&
                (frame->msg_id ==
                 UBX_ID_NAV_PVT))
            {
                if (GPS_UBX_DecodeNAVPVT(
                        frame,
                        &gps->latest_pvt))
                {
                    gps->pvt_ready =
                        true;
                }
            }
            else if (
                (frame->msg_class ==
                 UBX_CLASS_MON) &&
                (frame->msg_id ==
                 UBX_ID_MON_VER))
            {
                if (GPS_UBX_DecodeMONVER(
                        frame,
                        &gps->latest_mon_ver))
                {
                    gps->mon_ver_ready =
                        true;
                }
            }
            else if (
                (frame->msg_class ==
                 UBX_CLASS_ACK) &&
                (frame->msg_id ==
                 UBX_ID_ACK_ACK) &&
                (frame->length >= 2U))
            {
                gps->ack_class =
                    frame->payload[0];

                gps->ack_id =
                    frame->payload[1];

                gps->ack_status = 1;
            }
            else if (
                (frame->msg_class ==
                 UBX_CLASS_ACK) &&
                (frame->msg_id ==
                 UBX_ID_ACK_NAK) &&
                (frame->length >= 2U))
            {
                gps->ack_class =
                    frame->payload[0];

                gps->ack_id =
                    frame->payload[1];

                gps->ack_status = -1;
            }
        }
        else if (result ==
                 UBX_PARSE_CHECKSUM_ERROR)
        {
            gps->checksum_errors++;
        }
    }
}

bool GPS_UBX_GetNAVPVT(
    GPS_UBX_t *gps,
    GPS_UBX_NAV_PVT_t *pvt)
{
    if ((gps == NULL) ||
        (pvt == NULL) ||
        (!gps->pvt_ready))
    {
        return false;
    }

    *pvt = gps->latest_pvt;
    gps->pvt_ready = false;

    return true;
}

bool GPS_UBX_GetMONVER(
    GPS_UBX_t *gps,
    GPS_UBX_MON_VER_t *version)
{
    if ((gps == NULL) ||
        (version == NULL) ||
        (!gps->mon_ver_ready))
    {
        return false;
    }

    *version = gps->latest_mon_ver;
    gps->mon_ver_ready = false;

    return true;
}

void GPS_UBX_DebugPrint(
    GPS_UBX_t *gps,
    const char *text)
{
    if ((gps == NULL) ||
        (gps->debug_uart == NULL) ||
        (text == NULL))
    {
        return;
    }

    HAL_UART_Transmit(
        gps->debug_uart,
        (uint8_t *)text,
        (uint16_t)strlen(text),
        HAL_MAX_DELAY
    );
}

void GPS_UBX_PrintMONVER(
    GPS_UBX_t *gps,
    const GPS_UBX_MON_VER_t *version)
{
    char msg[120];
    uint8_t i;

    if ((gps == NULL) ||
        (version == NULL))
    {
        return;
    }

    snprintf(
        msg,
        sizeof(msg),
        "MON-VER payload = %u byte\r\n",
        (unsigned int)
        version->payload_length
    );

    GPS_UBX_DebugPrint(gps, msg);

    snprintf(
        msg,
        sizeof(msg),
        "SW : %s\r\n",
        version->software
    );

    GPS_UBX_DebugPrint(gps, msg);

    snprintf(
        msg,
        sizeof(msg),
        "HW : %s\r\n",
        version->hardware
    );

    GPS_UBX_DebugPrint(gps, msg);

    for (i = 0U;
         i < version->extension_count;
         i++)
    {
        snprintf(
            msg,
            sizeof(msg),
            "EXT: %s\r\n",
            version->extensions[i]
        );

        GPS_UBX_DebugPrint(gps, msg);
    }
}

void GPS_UBX_PrintNAVPVT(
    GPS_UBX_t *gps,
    const GPS_UBX_NAV_PVT_t *pvt)
{
    char msg[430];

    int64_t lat_abs;
    int64_t lon_abs;
    int64_t alt_abs;
    int64_t speed_abs;
    int64_t heading_abs;

    char lat_sign;
    char lon_sign;
    char alt_sign;
    char heading_sign;

    if ((gps == NULL) ||
        (pvt == NULL))
    {
        return;
    }

    lat_abs = pvt->latitude_raw;
    lon_abs = pvt->longitude_raw;
    alt_abs = pvt->hmsl_mm;
    speed_abs = pvt->ground_speed_mm_s;
    heading_abs = pvt->heading_raw;

    if (lat_abs < 0) lat_abs = -lat_abs;
    if (lon_abs < 0) lon_abs = -lon_abs;
    if (alt_abs < 0) alt_abs = -alt_abs;
    if (speed_abs < 0) speed_abs = -speed_abs;
    if (heading_abs < 0) heading_abs = -heading_abs;

    lat_sign =
        (pvt->latitude_raw < 0)
        ? '-' : '+';

    lon_sign =
        (pvt->longitude_raw < 0)
        ? '-' : '+';

    alt_sign =
        (pvt->hmsl_mm < 0)
        ? '-' : '+';

    heading_sign =
        (pvt->heading_raw < 0)
        ? '-' : '+';

    snprintf(
        msg,
        sizeof(msg),

        "PVT | "
        "%04u-%02u-%02u "
        "%02u:%02u:%02u | "
        "dateOK=%u | "
        "timeOK=%u | "
        "resolved=%u | "
        "fix=%u | "
        "fixOK=%u | "
        "SV=%u | "
        "LAT=%c%ld.%07ld | "
        "LON=%c%ld.%07ld | "
        "ALT=%c%ld.%03ld m | "
        "hAcc=%lu.%03lu m | "
        "Speed=%ld.%03ld m/s | "
        "Heading=%c%ld.%05ld deg\r\n",

        (unsigned int)pvt->year,
        (unsigned int)pvt->month,
        (unsigned int)pvt->day,

        (unsigned int)pvt->hour,
        (unsigned int)pvt->minute,
        (unsigned int)pvt->second,

        pvt->valid_date ? 1U : 0U,
        pvt->valid_time ? 1U : 0U,
        pvt->fully_resolved ? 1U : 0U,

        (unsigned int)pvt->fix_type,
        pvt->fix_ok ? 1U : 0U,
        (unsigned int)pvt->num_sv,

        lat_sign,
        (long)(lat_abs / 10000000LL),
        (long)(lat_abs % 10000000LL),

        lon_sign,
        (long)(lon_abs / 10000000LL),
        (long)(lon_abs % 10000000LL),

        alt_sign,
        (long)(alt_abs / 1000LL),
        (long)(alt_abs % 1000LL),

        (unsigned long)(
            pvt->hacc_mm / 1000UL
        ),
        (unsigned long)(
            pvt->hacc_mm % 1000UL
        ),

        (long)(speed_abs / 1000LL),
        (long)(speed_abs % 1000LL),

        heading_sign,
        (long)(heading_abs / 100000LL),
        (long)(heading_abs % 100000LL)
    );

    GPS_UBX_DebugPrint(gps, msg);
}

void GPS_UBX_PrintStats(
    GPS_UBX_t *gps)
{
    char msg[280];
    uint32_t ndtr = 0U;
    uint32_t uart_error = 0U;

    if (gps == NULL)
    {
        return;
    }

    if ((gps->gps_uart != NULL) &&
        (gps->gps_uart->hdmarx != NULL))
    {
        ndtr = __HAL_DMA_GET_COUNTER(
            gps->gps_uart->hdmarx
        );

        uart_error = HAL_UART_GetError(
            gps->gps_uart
        );
    }

    snprintf(
        msg,
        sizeof(msg),
        "[DMA] NDTR=%lu | RX=%lu | Frames=%lu | "
        "CKERR=%lu | DROP=%lu | RB_OVF=%lu | "
        "HT=%lu | TC=%lu | FLAG=%u | "
        "UART_ERR=0x%08lX | REC=%lu\r\n",
        (unsigned long)ndtr,
        (unsigned long)gps->received_bytes,
        (unsigned long)gps->parsed_frames,
        (unsigned long)gps->checksum_errors,
        (unsigned long)gps->dropped_bytes,
        (unsigned long)gps->ring.overflow_count,
        (unsigned long)gps->dma_half_events,
        (unsigned long)gps->dma_full_events,
        (unsigned int)gps->rx_data_flag,
        (unsigned long)uart_error,
        (unsigned long)gps->recovery_count
    );

    GPS_UBX_DebugPrint(gps, msg);
}

double GPS_UBX_LatitudeDeg(
    const GPS_UBX_NAV_PVT_t *pvt)
{
    return ((double)
            pvt->latitude_raw)
           * 1e-7;
}

double GPS_UBX_LongitudeDeg(
    const GPS_UBX_NAV_PVT_t *pvt)
{
    return ((double)
            pvt->longitude_raw)
           * 1e-7;
}

double GPS_UBX_AltitudeMSL_M(
    const GPS_UBX_NAV_PVT_t *pvt)
{
    return ((double)
            pvt->hmsl_mm)
           / 1000.0;
}

double GPS_UBX_GroundSpeed_MPS(
    const GPS_UBX_NAV_PVT_t *pvt)
{
    return ((double)
            pvt->ground_speed_mm_s)
           / 1000.0;
}

double GPS_UBX_HeadingDeg(
    const GPS_UBX_NAV_PVT_t *pvt)
{
    return ((double)
            pvt->heading_raw)
           * 1e-5;
}

double GPS_UBX_HAcc_M(
    const GPS_UBX_NAV_PVT_t *pvt)
{
    return ((double)
            pvt->hacc_mm)
           / 1000.0;
}
