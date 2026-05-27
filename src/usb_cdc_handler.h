// =============================================================================
// usb_cdc_handler.h — USB Host CDC-ACM handler (P4 ↔ S3 via USB-C)
// Reads binary protocol from S3 HWCDC over USB-OTG Host port
// =============================================================================
#pragma once

#include <stdint.h>
#include <stddef.h>

// Initialize USB Host Library + CDC ACM driver on USB-OTG port
void usb_cdc_init(void);

// Process USB Host events — call from main loop
void usb_cdc_process(void);

// Check if S3 CDC device is connected
bool usb_cdc_connected(void);

// Bytes available in USB CDC receive buffer
int usb_cdc_available(void);

// Read one byte from USB CDC buffer (-1 if empty)
int usb_cdc_read(void);

// Send data to S3 via USB CDC
size_t usb_cdc_write(const uint8_t* data, size_t len);

// Get diagnostic string for USB CDC state (for UART-DBG)
// Returns: "init=X conn=X dev=X open=X disc=X last_err=X"
const char* usb_cdc_status_str(void);

// Compact counters for visible UI diagnostics.
void usb_cdc_get_diag(int* detected, int* connects, int* disconnects, int* attempts);
