#ifndef CRSF_H
#define CRSF_H

#include <stdint.h>
#include "crsf_protocol.h"  // ✅ Tumhara existing protocol

// ============= CRSF DATA STRUCTURE =============
struct crsfData_t {
    uint16_t channels[16];      // Raw CRSF format (172-1811)
    bool valid;                 // ✅ Validity flag only!
};

// ============= CRSF FUNCTIONS =============

// Initialize CRSF parser
void crsfInit();

// Process one byte from receiver
// Returns: true if frame complete, false otherwise
bool crsfProcessByte(uint8_t c, crsfData_t* outData);

#endif