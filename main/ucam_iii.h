#include <stdio.h>
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "soc/uart_struct.h"

#define DEBUG_OUTPUT

#define COMMAND_START 0xAA

#define INIT_ID 0x01
#define INIT_P2_RAW_8BIT_GRAYSCALE 0x03
#define INIT_P2_RAW_16BIT_COLOUR_CRYCBY 0x08
#define INIT_P2_RAW_16BIT_COLOUR_RGB 0x06
#define INIT_P2_JPEG 0x07
#define INIT_P3_RAW_80X60 0x01
#define INIT_P3_RAW_160X120 0x03
#define INIT_P3_RAW_128X128 0x09
#define INIT_P3_RAW_128X96 0x0B
#define INIT_P4_JPEG_160X128 0x03
#define INIT_P4_JPEG_320X240 0x05
#define INIT_P4_JPEG_640X480 0x07

#define GET_PICTURE_ID 0x04
#define GET_PICTURE_P1_SNAPSHOT_MODE 0x01
#define GET_PICTURE_P1_RAW_MODE 0x02
#define GET_PICTURE_P1_JPEG_MODE 0x05

#define SNAPSHOT_ID 0x05
#define SNAPSHOT_P1_JPEG 0x00
#define SNAPSHOT_P1_RAW 0x01

#define SET_PKG_SIZE_ID 0x06
#define SET_PKG_SIZE_P1 0x08

#define RESET_ID 0x08

#define DATA_ID 0x0A
#define DATA_P1_SNAPSHOT 0x01
#define DATA_P1_RAW 0x02
#define DATA_P1_JPEG 0x05

#define SYNC_ID 0x0D

#define ACK_ID 0x0E

void print_buffer_as_hex_compact(uint8_t* address, int length);
void print_buffer_as_hex(uint8_t* address, int length);
void print_buffer_as_char(uint8_t* address, int length);

bool receiveAck(uint8_t command_id, int ack_timeout_ms);
bool receiveSync(int ack_timeout_ms);

void sendPacketAck(uint8_t id_byte_0, uint8_t id_byte_1);
bool sendCommand(uint8_t command_id, uint8_t param1, uint8_t param2, uint8_t param3, uint8_t param4, int ack_timeout_ms);
void sendAck(uint8_t command_id);

bool syncCamera();

uint32_t request_image(uint8_t image_type);
bool receive_jpeg_image(uint32_t image_size, uint16_t packet_size, uint8_t* dest_buffer);
bool receive_raw_image(uint16_t rec_buffer_size, uint8_t* dest_buffer);

