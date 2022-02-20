/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "soc/uart_struct.h"

#define ECHO_TEST_TXD (17)
#define ECHO_TEST_RXD (16)
#define BUF_SIZE (4096)

#ifdef CONFIG_IDF_TARGET_ESP32
#define CHIP_NAME "ESP32"
#endif

#ifdef CONFIG_IDF_TARGET_ESP32S2BETA
#define CHIP_NAME "ESP32-S2 Beta"
#endif

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
#define SNAPSHOT_P2_RAW 0x01

#define SET_PKG_SIZE_ID 0x06
#define SET_PKG_SIZE_P1 0x08

#define RESET_ID 0x08

#define DATA_ID 0x0A
#define DATA_P1_SNAPSHOT 0x01
#define DATA_P1_RAW 0x02
#define DATA_P1_JPEG 0x05

#define SYNC_ID 0x0D

#define ACK_ID 0x0E

/*  print_buffer_as_hex_compact()//{ */

void print_buffer_as_hex_compact(uint8_t* address, int length) {
#ifndef DEBUG_OUTPUT
  return;
#endif
  for (int i = 0; i < length; i++) {
    printf("%x", *(address + i));
    if (i % 64 == 0 && i != 0) {
      printf("\n");
    }
  }
}

//}

/* print_buffer_as_hex() //{ */

void print_buffer_as_hex(uint8_t* address, int length) {
#ifndef DEBUG_OUTPUT
  return;
#endif
  for (int i = 0; i < length; i++) {
    printf(" %X", *(address + i));
    if (i % 64 == 0 && i != 0) {
      printf("\n");
    }
  }
}

//}

/* print_buffer_as_char() //{ */

void print_buffer_as_char(uint8_t* address, int length) {
#ifndef DEBUG_OUTPUT
  return;
#endif
  printf("message as char: ");
  for (int i = 0; i < length; i++) {
    printf("%c", *(address + i));
  }
  printf("\n");
}

//}

/* receiveAck() //{ */

bool receiveAck(uint8_t command_id, int ack_timeout_ms) {

  bool ret_val = false;

  uint8_t receive_buf[6];
  int     len = uart_read_bytes(UART_NUM_1, receive_buf, 6, ack_timeout_ms / portTICK_RATE_MS);

  if (receive_buf[0] == COMMAND_START && receive_buf[1] == ACK_ID && receive_buf[2] == command_id && len == 6) {
    ret_val = true;
  }

#ifdef DEBUG_OUTPUT
  printf("REC %i: ", len);
  print_buffer_as_hex(receive_buf, len);
  printf("   ACK: ");
  printf(ret_val ? "true" : "false");
  printf("\n---------------------------------\n");
#endif

  return ret_val;
}

//}

/* receiveSync() //{ */

bool receiveSync(int ack_timeout_ms) {

  bool ret_val = false;

  uint8_t receive_buf[6];
  int     len = uart_read_bytes(UART_NUM_1, receive_buf, 6, ack_timeout_ms / portTICK_RATE_MS);

  if (receive_buf[0] == COMMAND_START && receive_buf[1] == SYNC_ID && len == 6) {
    ret_val = true;
  }

#ifdef DEBUG_OUTPUT
  printf("REC %i: \n", len);
  print_buffer_as_hex(receive_buf, len);
  printf("SYNC: ");
  printf(ret_val ? "true" : "false");
  printf("\n---------------------------------\n");
#endif

  return ret_val;
}

//}

/* receiveData() //{ */

bool receiveData(int ack_timeout_ms) {

  bool ret_val = false;

  uint8_t receive_buf[6];
  int     len = uart_read_bytes(UART_NUM_1, receive_buf, 6, ack_timeout_ms / portTICK_RATE_MS);

  if (receive_buf[0] == COMMAND_START && receive_buf[1] == DATA_ID && len == 6) {
    ret_val = true;
  }

  uint32_t data_size = 0;

  data_size = (receive_buf[3] << 16) + (receive_buf[4] << 8) + receive_buf[5];

  printf("DATA_SIZE: %i  %X %X %X\n", data_size, receive_buf[3], receive_buf[4], receive_buf[5]);
#ifdef DEBUG_OUTPUT
  printf("REC %i: \n", len);
  print_buffer_as_hex(receive_buf, len);
  printf("DATA: ");
  printf(ret_val ? "true" : "false");
  printf("\n---------------------------------\n");
#endif

  return ret_val;
}

//}

/* sendPacketAck() //{ */

void sendPacketAck(uint8_t id_byte_0, uint8_t id_byte_1) {

  uint8_t command[6];
  command[0] = COMMAND_START;
  command[1] = ACK_ID;
  command[2] = 0x00;
  command[3] = 0x00;
  command[4] = id_byte_1;
  command[5] = id_byte_0;

#ifdef DEBUG_OUTPUT
  printf("SENT: ");
  print_buffer_as_hex(command, 6);
  printf("\n---------------------------------\n");
#endif

  uart_write_bytes(UART_NUM_1, (const char*)command, 6);
}

//}

/* receiveImagePacket() //{ */

bool receiveImagePacket(int ack_timeout_ms) {

  bool ret_val = false;

  uint8_t receive_buf[512];
  int     len = uart_read_bytes(UART_NUM_1, receive_buf, 512, ack_timeout_ms / portTICK_RATE_MS);

  if (len == 512) {
    printf("got 512 bytes, ID: ");
    print_buffer_as_hex(receive_buf, len);
    sendPacketAck(receive_buf[1], receive_buf[0]);
    ret_val = true;
  } else if (len != 0) {
    printf("got %i bytes, ID: \n", len);
    print_buffer_as_hex(receive_buf, len);
  }

  return ret_val;
}

//}

/* sendCommand() //{ */

bool sendCommand(uint8_t command_id, uint8_t param1, uint8_t param2, uint8_t param3, uint8_t param4, int ack_timeout_ms) {

  uint8_t command[6];
  command[0] = COMMAND_START;  // first byte is always 0xAA
  command[1] = command_id;
  command[2] = param1;
  command[3] = param2;
  command[4] = param3;
  command[5] = param4;

#ifdef DEBUG_OUTPUT
  printf("---------------------------------\n");
  printf("SENT: ");
  print_buffer_as_hex(command, 6);
  printf("\n");
#endif

  uart_write_bytes(UART_NUM_1, (const char*)command, 6);

  return receiveAck(command_id, ack_timeout_ms);
}

//}

/* sendAck() //{ */

void sendAck(uint8_t command_id) {

  uint8_t command[6];
  command[0] = COMMAND_START;
  command[1] = ACK_ID;
  command[2] = command_id;
  command[3] = 0x00;
  command[4] = 0x00;
  command[5] = 0x00;

#ifdef DEBUG_OUTPUT
  printf("---------------------------------\n");
  printf("SENT: ");
  print_buffer_as_hex(command, 6);
  printf("\n---------------------------------\n");
#endif

  uart_write_bytes(UART_NUM_1, (const char*)command, 6);
}

//}

/* syncCamera //{ */

bool syncCamera() {

  int  num_attempts = 0;
  bool got_ack      = false;

  while (!got_ack) {

    if (sendCommand(SYNC_ID, 0x00, 0x00, 0x00, 0x00, 5)) {
      got_ack = true;
      break;
    }

    num_attempts++;

    if (num_attempts > 60) {

#ifdef DEBUG_OUTPUT
      printf("Did not get a reply after 60 SYNC attempts\n");
#endif

      return false;
    }
  }

  if (!receiveSync(100)) {

#ifdef DEBUG_OUTPUT
    printf("Did not get a SYNC response from camera\n");
#endif
    return false;
  }

  printf("Succesfully synced\n");
  sendAck(SYNC_ID);
  printf("Succesfully synced and sync id sent back, waiting 2000ms as per datasheet\n");

  vTaskDelay(2000 / portTICK_PERIOD_MS);
  return true;
}

//}

/* app_main() //{ */

void app_main(void) {

  const int     uart_num    = UART_NUM_1;
  uart_config_t uart_config = {
      .baud_rate           = 115200,
      .data_bits           = UART_DATA_8_BITS,
      .parity              = UART_PARITY_DISABLE,
      .stop_bits           = UART_STOP_BITS_1,
      .flow_ctrl           = UART_HW_FLOWCTRL_DISABLE,
      .rx_flow_ctrl_thresh = 122,
  };

  // Configure UART1 parameters
  uart_param_config(uart_num, &uart_config);
  // Set UART1 pins(TX: IO4, RX: I05)
  uart_set_pin(uart_num, ECHO_TEST_TXD, ECHO_TEST_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  // Install UART driver (we don't need an event queue here)
  // In this example we don't even use a buffer for sending data.
  uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, 0);

  printf("aaand a one\n");
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  printf("aaand a two\n");
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  printf("aaand a three\n");
  vTaskDelay(2000 / portTICK_PERIOD_MS);

  if (syncCamera()) {
    printf("camera synced\n");
  } else {
    while (1) {
      printf("camera failed to sync\n");
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }

  printf("sending RESET\n");
  sendCommand(RESET_ID, 0x00, 0x00, 0x00, 0x00, 100);
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  if (syncCamera()) {
    printf("camera synced\n");
  } else {
    while (1) {
      printf("camera failed to sync\n");
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }

  printf("sending init\n");
  sendCommand(INIT_ID, 0x00, INIT_P2_JPEG, 0x07, INIT_P4_JPEG_640X480, 100);
  vTaskDelay(100 / portTICK_PERIOD_MS);

  printf("sending package size 256 bytes\n");
  sendCommand(SET_PKG_SIZE_ID, SET_PKG_SIZE_P1, 0x00, 0x02, 0x09, 100);
  vTaskDelay(100 / portTICK_PERIOD_MS);

  printf("sending snapshot mode\n");
  sendCommand(SNAPSHOT_ID, SNAPSHOT_P1_JPEG, 0x00, 0x00, 0x00, 100);
  vTaskDelay(100 / portTICK_PERIOD_MS);

  printf("sending get picture\n");
  sendCommand(GET_PICTURE_ID, GET_PICTURE_P1_SNAPSHOT_MODE, 0x00, 0x00, 0x00, 100);
  vTaskDelay(100 / portTICK_PERIOD_MS);

  /* if (receiveData(100)) { */
  receiveData(500);
  printf("got DATA, sending ACK\n");
  sendAck(DATA_ID);
  /* } */


  while (1) {
    if (receiveImagePacket(100)) {
      printf("receiving image packet\n");
    }
  }
}

//}
