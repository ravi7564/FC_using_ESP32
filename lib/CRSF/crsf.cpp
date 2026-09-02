#include "crsf.h"
#include "crsf_protocol.h"
#include <cstring>

static struct {
    uint8_t bytes[CRSF_FRAME_SIZE_MAX];
    uint8_t framePos;
} crsfState = {0};

uint8_t crsfCalculateCRC(const uint8_t* data, uint8_t len)
{
    uint8_t crc = 0;

    for (uint8_t i = 0; i < len; i++)
    {
        crc ^= data[i];

        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ 0xD5;
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}

void crsfInit()
{
    crsfState.framePos = 0;
    std::memset(crsfState.bytes, 0, CRSF_FRAME_SIZE_MAX);
}

// ============= CRSF Frame Processing & Validation =============
bool crsfProcessByte(uint8_t c, crsfData_t* outData)
{
    if (outData == nullptr)
        return false;

    // ✅ Default: data invalid
    outData->valid = false;

    // -------------------------------------------------
    // 1. Wait for CRSF sync byte
    // -------------------------------------------------
    if (crsfState.framePos == 0)
    {
        if (c != CRSF_SYNC_BYTE)
            return false;
    }

    // -------------------------------------------------
    // 2. Prevent buffer overflow
    // -------------------------------------------------
    if (crsfState.framePos >= CRSF_FRAME_SIZE_MAX)
    {
        crsfState.framePos = 0;
        return false;
    }

    crsfState.bytes[crsfState.framePos++] = c;

    // Need at least: [SYNC][LENGTH]
    if (crsfState.framePos < 2)
        return false;

    // -------------------------------------------------
    // 3. Read frame length
    // -------------------------------------------------
    const uint8_t frameLength = crsfState.bytes[1];

    if (frameLength < 2 ||
        frameLength > (CRSF_PAYLOAD_SIZE_MAX + 2))
    {
        crsfState.framePos = 0;
        return false;
    }

    // Total frame size: SYNC + LENGTH + frameLength
    const uint8_t expectedFrameSize = frameLength + 2;

    if (crsfState.framePos < expectedFrameSize)
        return false;

    // -------------------------------------------------
    // 4. CRC validation
    // -------------------------------------------------
    const uint8_t crcReceived = crsfState.bytes[crsfState.framePos - 1];
    const uint8_t crcCalculated = crsfCalculateCRC(
        &crsfState.bytes[2],
        crsfState.framePos - 3
    );

    if (crcCalculated != crcReceived)
    {
        crsfState.framePos = 0;
        return false;
    }

    // -------------------------------------------------
    // 5. Extract channels (only if RC_CHANNELS frame)
    // -------------------------------------------------
    const uint8_t frameType = crsfState.bytes[2];

    if (frameType == CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
    {
        const crsfPayloadRcChannelsPacked_s* packed =
            reinterpret_cast<const crsfPayloadRcChannelsPacked_s*>(&crsfState.bytes[3]);

        // ✅ Extract channels (raw CRSF format 172-1811)
        outData->channels[0] = packed->chan0;
        outData->channels[1] = packed->chan1;
        outData->channels[2] = packed->chan2;
        outData->channels[3] = packed->chan3;
        outData->channels[4] = packed->chan4;
        outData->channels[5] = packed->chan5;
        outData->channels[6] = packed->chan6;
        outData->channels[7] = packed->chan7;
        outData->channels[8] = packed->chan8;
        outData->channels[9] = packed->chan9;
        outData->channels[10] = packed->chan10;
        outData->channels[11] = packed->chan11;
        outData->channels[12] = packed->chan12;
        outData->channels[13] = packed->chan13;
        outData->channels[14] = packed->chan14;
        outData->channels[15] = packed->chan15;

        // Mark received RC channels packet as valid
        outData->valid = true;
    }

    // -------------------------------------------------
    // 6. Frame successfully processed
    // -------------------------------------------------
    crsfState.framePos = 0;
    return true;
}