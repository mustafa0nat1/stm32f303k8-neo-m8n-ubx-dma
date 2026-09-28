#ifndef UBX_H
#define UBX_H

#include <stdint.h>
#include <stdbool.h>

#define UBX_SYNC_1               0xB5U
#define UBX_SYNC_2               0x62U

#define UBX_CLASS_NAV            0x01U
#define UBX_CLASS_ACK            0x05U
#define UBX_CLASS_CFG            0x06U
#define UBX_CLASS_MON            0x0AU

#define UBX_ID_NAV_PVT           0x07U
#define UBX_ID_MON_VER           0x04U
#define UBX_ID_ACK_NAK           0x00U
#define UBX_ID_ACK_ACK           0x01U
#define UBX_ID_CFG_MSG           0x01U

#define UBX_MAX_PAYLOAD          256U

typedef struct
{
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t length;
    uint8_t payload[UBX_MAX_PAYLOAD];
    uint8_t ck_a;
    uint8_t ck_b;
} UBX_Frame_t;

typedef enum
{
    UBX_PARSE_NONE = 0,
    UBX_PARSE_FRAME_READY,
    UBX_PARSE_CHECKSUM_ERROR,
    UBX_PARSE_PAYLOAD_TOO_LARGE
} UBX_ParseResult_t;

typedef enum
{
    UBX_STATE_SYNC1 = 0,
    UBX_STATE_SYNC2,
    UBX_STATE_CLASS,
    UBX_STATE_ID,
    UBX_STATE_LEN1,
    UBX_STATE_LEN2,
    UBX_STATE_PAYLOAD,
    UBX_STATE_CK_A,
    UBX_STATE_CK_B
} UBX_ParserState_t;

typedef struct
{
    UBX_ParserState_t state;
    UBX_Frame_t frame;
    uint16_t payload_index;
    uint8_t calc_ck_a;
    uint8_t calc_ck_b;
    uint8_t received_ck_a;
    bool discard_payload;

    uint32_t valid_frames;
    uint32_t checksum_errors;
    uint32_t oversized_frames;
} UBX_Parser_t;

void UBX_Checksum(const uint8_t *data,
                  uint16_t length,
                  uint8_t *ck_a,
                  uint8_t *ck_b);

uint16_t UBX_BuildPacket(uint8_t msg_class,
                         uint8_t msg_id,
                         const uint8_t *payload,
                         uint16_t payload_length,
                         uint8_t *output,
                         uint16_t output_size);

uint16_t UBX_BuildPoll(uint8_t msg_class,
                       uint8_t msg_id,
                       uint8_t *output,
                       uint16_t output_size);

void UBX_ParserInit(UBX_Parser_t *parser);
UBX_ParseResult_t UBX_ParseByte(UBX_Parser_t *parser, uint8_t byte);
const UBX_Frame_t *UBX_GetFrame(const UBX_Parser_t *parser);

uint16_t UBX_ReadU16(const uint8_t *data);
uint32_t UBX_ReadU32(const uint8_t *data);
int32_t UBX_ReadI32(const uint8_t *data);

#endif
