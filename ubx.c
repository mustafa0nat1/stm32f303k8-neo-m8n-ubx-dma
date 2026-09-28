#include "ubx.h"

void UBX_Checksum(const uint8_t *data,
                  uint16_t length,
                  uint8_t *ck_a,
                  uint8_t *ck_b)
{
    uint16_t i;

    *ck_a = 0U;
    *ck_b = 0U;

    for (i = 0U; i < length; i++)
    {
        *ck_a = (uint8_t)(*ck_a + data[i]);
        *ck_b = (uint8_t)(*ck_b + *ck_a);
    }
}

uint16_t UBX_BuildPacket(uint8_t msg_class,
                         uint8_t msg_id,
                         const uint8_t *payload,
                         uint16_t payload_length,
                         uint8_t *output,
                         uint16_t output_size)
{
    uint16_t i;
    uint16_t total_length;
    uint8_t ck_a;
    uint8_t ck_b;

    total_length = (uint16_t)(payload_length + 8U);

    if ((output == 0) || (output_size < total_length))
    {
        return 0U;
    }

    output[0] = UBX_SYNC_1;
    output[1] = UBX_SYNC_2;
    output[2] = msg_class;
    output[3] = msg_id;
    output[4] = (uint8_t)(payload_length & 0xFFU);
    output[5] = (uint8_t)((payload_length >> 8) & 0xFFU);

    for (i = 0U; i < payload_length; i++)
    {
        output[6U + i] = payload[i];
    }

    UBX_Checksum(
        &output[2],
        (uint16_t)(4U + payload_length),
        &ck_a,
        &ck_b
    );

    output[6U + payload_length] = ck_a;
    output[7U + payload_length] = ck_b;

    return total_length;
}

uint16_t UBX_BuildPoll(uint8_t msg_class,
                       uint8_t msg_id,
                       uint8_t *output,
                       uint16_t output_size)
{
    return UBX_BuildPacket(
        msg_class,
        msg_id,
        0,
        0U,
        output,
        output_size
    );
}

static void UBX_ChecksumAdd(UBX_Parser_t *parser, uint8_t byte)
{
    parser->calc_ck_a = (uint8_t)(parser->calc_ck_a + byte);
    parser->calc_ck_b = (uint8_t)(parser->calc_ck_b + parser->calc_ck_a);
}

void UBX_ParserInit(UBX_Parser_t *parser)
{
    parser->state = UBX_STATE_SYNC1;
    parser->payload_index = 0U;
    parser->calc_ck_a = 0U;
    parser->calc_ck_b = 0U;
    parser->received_ck_a = 0U;
    parser->discard_payload = false;

    parser->valid_frames = 0U;
    parser->checksum_errors = 0U;
    parser->oversized_frames = 0U;
}

UBX_ParseResult_t UBX_ParseByte(UBX_Parser_t *parser, uint8_t byte)
{
    switch (parser->state)
    {
        case UBX_STATE_SYNC1:
            if (byte == UBX_SYNC_1)
            {
                parser->state = UBX_STATE_SYNC2;
            }
            break;

        case UBX_STATE_SYNC2:
            if (byte == UBX_SYNC_2)
            {
                parser->state = UBX_STATE_CLASS;
                parser->payload_index = 0U;
                parser->calc_ck_a = 0U;
                parser->calc_ck_b = 0U;
                parser->discard_payload = false;
            }
            else if (byte == UBX_SYNC_1)
            {
                parser->state = UBX_STATE_SYNC2;
            }
            else
            {
                parser->state = UBX_STATE_SYNC1;
            }
            break;

        case UBX_STATE_CLASS:
            parser->frame.msg_class = byte;
            UBX_ChecksumAdd(parser, byte);
            parser->state = UBX_STATE_ID;
            break;

        case UBX_STATE_ID:
            parser->frame.msg_id = byte;
            UBX_ChecksumAdd(parser, byte);
            parser->state = UBX_STATE_LEN1;
            break;

        case UBX_STATE_LEN1:
            parser->frame.length = byte;
            UBX_ChecksumAdd(parser, byte);
            parser->state = UBX_STATE_LEN2;
            break;

        case UBX_STATE_LEN2:
            parser->frame.length |= ((uint16_t)byte << 8);
            UBX_ChecksumAdd(parser, byte);
            parser->payload_index = 0U;

            if (parser->frame.length > UBX_MAX_PAYLOAD)
            {
                parser->discard_payload = true;
            }

            parser->state = (parser->frame.length == 0U)
                            ? UBX_STATE_CK_A
                            : UBX_STATE_PAYLOAD;
            break;

        case UBX_STATE_PAYLOAD:
            UBX_ChecksumAdd(parser, byte);

            if (!parser->discard_payload)
            {
                parser->frame.payload[parser->payload_index] = byte;
            }

            parser->payload_index++;

            if (parser->payload_index >= parser->frame.length)
            {
                parser->state = UBX_STATE_CK_A;
            }
            break;

        case UBX_STATE_CK_A:
            parser->received_ck_a = byte;
            parser->state = UBX_STATE_CK_B;
            break;

        case UBX_STATE_CK_B:
            parser->frame.ck_a = parser->received_ck_a;
            parser->frame.ck_b = byte;
            parser->state = UBX_STATE_SYNC1;

            if ((parser->received_ck_a != parser->calc_ck_a) ||
                (byte != parser->calc_ck_b))
            {
                parser->checksum_errors++;
                return UBX_PARSE_CHECKSUM_ERROR;
            }

            if (parser->discard_payload)
            {
                parser->oversized_frames++;
                return UBX_PARSE_PAYLOAD_TOO_LARGE;
            }

            parser->valid_frames++;
            return UBX_PARSE_FRAME_READY;

        default:
            parser->state = UBX_STATE_SYNC1;
            break;
    }

    return UBX_PARSE_NONE;
}

const UBX_Frame_t *UBX_GetFrame(const UBX_Parser_t *parser)
{
    return &parser->frame;
}

uint16_t UBX_ReadU16(const uint8_t *data)
{
    return (uint16_t)data[0] |
           ((uint16_t)data[1] << 8);
}

uint32_t UBX_ReadU32(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

int32_t UBX_ReadI32(const uint8_t *data)
{
    return (int32_t)UBX_ReadU32(data);
}
