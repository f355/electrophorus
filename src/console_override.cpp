#include "mbed.h"

// Redirect printf/stdio to the UART not used by comms, so logging and comms never share a UART.

#ifndef EPHO_COMMS_UART
#define EPHO_COMMS_UART 2
#endif

#if EPHO_COMMS_UART == 0
static constexpr PinName LOG_TX_PIN = P2_8;  // UART2 TX
static constexpr PinName LOG_RX_PIN = P2_9;  // UART2 RX
#elif EPHO_COMMS_UART == 2
static constexpr PinName LOG_TX_PIN = P0_2;  // UART0 TX
static constexpr PinName LOG_RX_PIN = P0_3;  // UART0 RX
#else
#error "EPHO_COMMS_UART must be 0 or 2"
#endif

// Use UnbufferedSerial for minimal overhead; baud comes from MBED config (mbed_app.json5)
static mbed::UnbufferedSerial log_serial(LOG_TX_PIN, LOG_RX_PIN, MBED_CONF_PLATFORM_STDIO_BAUD_RATE);

namespace mbed {
FileHandle* mbed_override_console(int fd) {
    (void)fd;
    return &log_serial;
}
}  // namespace mbed

