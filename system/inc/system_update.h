
#ifndef SYSTEM_UPDATE_H
#define	SYSTEM_UPDATE_H

#ifdef __cplusplus
extern "C" {
#endif    

typedef struct Stream Stream;
#include <stdint.h>

typedef bool (*ymodem_serial_flash_update_handler)(Stream *serialObj, uint32_t sFlashAddress);
void set_ymodem_serial_flash_update_handler(ymodem_serial_flash_update_handler handler);

void set_start_dfu_flasher_serial_speed(uint32_t speed);
void set_start_ymodem_flasher_serial_speed(uint32_t speed);

bool system_serialSaveFile(Stream *serialObj, uint32_t sFlashAddress);

bool system_serialFirmwareUpdate(Stream *serialObj);

void system_lineCodingBitRateHandler(uint32_t bitrate);

#ifdef __cplusplus
}
#endif


#endif	/* SYSTEM_UPDATE_H */
