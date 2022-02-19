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
#define BUF_SIZE (38400)

#ifdef CONFIG_IDF_TARGET_ESP32
#define CHIP_NAME "ESP32"
#endif

#ifdef CONFIG_IDF_TARGET_ESP32S2BETA
#define CHIP_NAME "ESP32-S2 Beta"
#endif

#define DEBUG_OUTPUT


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

#define SET_PKG_SIZE_ID 0x06
#define SET_PKG_SIZE_P1 0x08

#define ID_GET_SNAPSHOT_ID 0x05

#define SYNC_ID 0x0D

void print_buffer_as_hex_compact(uint8_t* address, int length) {
#ifndef DEBUG_OUTPUT
  return;
#endif
  printf("message as hexa: ");
  for (int i = 0; i < length; i++) {
    printf("%x", *(address + i));
  }
  printf("\n");
}
void print_buffer_as_hex(uint8_t* address, int length) {
#ifndef DEBUG_OUTPUT
  return;
#endif
  printf("message as hexa: ");
  for (int i = 0; i < length; i++) {
    printf("%X ", *(address + i));
  }
  printf("\n");
}
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

bool sendCommand(uint8_t command_id, uint8_t param1, uint8_t param2, uint8_t param3, uint8_t param4) {

  bool ret_val = false;

  uint8_t command[6];
  command[0] = 0xAA;  // first byte is always 0xAA
  command[1] = command_id;
  command[2] = param1;
  command[3] = param2;
  command[4] = param3;
  command[5] = param4;
#ifdef DEBUG_OUTPUT
  printf("---------------------------------\n");
  printf("command sent: \n");
  print_buffer_as_hex(command, 6);
  printf("-----\n");
#endif
  uart_write_bytes(UART_NUM_1, (const char*)command, 6);

  uint8_t response_buf[6];
  int     len = uart_read_bytes(UART_NUM_1, response_buf, 6, 200 / portTICK_RATE_MS);

  if (response_buf[0] == 0xAA && response_buf[1] == 0x0E && response_buf[2] == command_id) {
    ret_val = true;
  }

#ifdef DEBUG_OUTPUT
  printf("received %i: \n", len);
  print_buffer_as_hex(response_buf, len);
  printf("ACK: ");
  printf(ret_val ? "true" : "false");
  printf("\n---------------------------------\n");
#endif

  return ret_val;
}

bool sendCommandUntilAck(uint8_t command_id, uint8_t param1, uint8_t param2, uint8_t param3, uint8_t param4, int num_attempts, int timeout_ms) {
  bool    ret_val = false;
  uint8_t command[6];
  command[0] = 0xAA;  // first byte is always 0xAA
  command[1] = command_id;
  command[2] = param1;
  command[3] = param2;
  command[4] = param3;
  command[5] = param4;
#ifdef DEBUG_OUTPUT
  printf("---------------------------------\n");
  printf("command sent: \n");
  print_buffer_as_hex(command, 6);
  printf("-----\n");
#endif
  uint8_t response_buf[6];

  bool got_ack         = false;
  bool timeout_reached = false;
  int  num_calls       = 0;

  while (!(got_ack || timeout_reached)) {
    printf("call\n");
    uart_write_bytes(UART_NUM_1, (const char*)command, 6);

    int len = uart_read_bytes(UART_NUM_1, response_buf, 6, (timeout_ms + num_calls) / portTICK_RATE_MS);
    printf("len: %i\n", len);
    if (len == 6) {
      print_buffer_as_hex(response_buf, 6);
    }

    num_calls++;
    if (num_calls > num_attempts) {
      timeout_reached = true;
    }

    if (response_buf[0] == 0xAA && response_buf[1] == 0x0E && response_buf[2] == command_id && len == 6) {
      got_ack = true;
      ret_val = true;
    }
  }

  printf("ACK: ");
  printf(ret_val ? "true" : "false");
  printf("\n---------------------------------\n");
  return ret_val;
}

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
  uint8_t* data = (uint8_t*)malloc(BUF_SIZE);

  printf("aaand a one\n");
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  printf("aaand a two\n");
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  printf("aaand a three\n");
  vTaskDelay(2000 / portTICK_PERIOD_MS);

  sendCommandUntilAck(SYNC_ID, 0x00, 0x00, 0x00, 0x00, 100, 5);
  printf("comms established, waiting 2 secs\n");
  vTaskDelay(2000 / portTICK_PERIOD_MS);

  sendCommandUntilAck(INIT_ID, 0x00, INIT_P2_RAW_8BIT_GRAYSCALE, INIT_P3_RAW_80X60, INIT_P4_JPEG_160X128, 5, 200);
  vTaskDelay(100 / portTICK_PERIOD_MS);

  sendCommandUntilAck(SET_PKG_SIZE_ID, SET_PKG_SIZE_P1, 0x00, 0x40, 0x00, 5, 200);
  vTaskDelay(100 / portTICK_PERIOD_MS);

  sendCommandUntilAck(GET_PICTURE_ID, GET_PICTURE_P1_RAW_MODE, 0x00, 0x00, 0x00, 5, 200);

  while (1) {
    // Read data from UART
    int len = uart_read_bytes(uart_num, data, BUF_SIZE, 500 / portTICK_RATE_MS);
    if (len > 0) {
      printf("message\n");
      printf("got: %i\n", *data);
      printf("got length: %i\n", len);
      print_buffer_as_hex_compact(data, len);
      /* print_buffer_as_char(data, len); */
      /* uart_write_bytes(uart_num, (const char*)reply, 4); */
    }
    /* // Write data back to UART */
    /* vTaskDelay(1000 / portTICK_PERIOD_MS); */
    /* printf("snooze\n"); */
  }
}
