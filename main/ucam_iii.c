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
#define BUF_SIZE (1024)

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
    printf("%02X", *(address + i));
    if (i % 512 == 0) {
      vTaskDelay(10 / portTICK_PERIOD_MS);
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
  /* printf("  SENT: "); */
  /* print_buffer_as_hex(command, 6); */
  /* printf("\n---------------------------------\n"); */
#endif

  uart_write_bytes(UART_NUM_1, (const char*)command, 6);
}

//}

/* receiveImagePacket() //{ */

uint16_t receiveImagePacket(int ack_timeout_ms, uint8_t* img_buffer, uint16_t offset) {

  uint16_t image_data_size = 0;

  uint8_t receive_buf[512];
  int     len = uart_read_bytes(UART_NUM_1, receive_buf, 512, ack_timeout_ms / portTICK_RATE_MS);

  if (len > 4) {
    image_data_size = (receive_buf[3] << 8) + receive_buf[2];
    /* printf("got %i bytes, data size: %i ID: ", len, image_data_size); */
    /* print_buffer_as_hex(receive_buf, len); */
    /* printf("\n"); */
    for (int i = 4; i < len - 2; i++) {
      img_buffer[i + offset - 4] = receive_buf[i];
    }
    sendPacketAck(receive_buf[1], receive_buf[0]);
  } else if (len != 0) {
    /* printf("got %i bytes, ID: \n", len); */
    /* print_buffer_as_hex(receive_buf, len); */
  }

  return image_data_size;
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

/* request_image() //{ */

uint32_t request_image(uint8_t image_type) {

  uint32_t ret_val = 0;

  sendCommand(GET_PICTURE_ID, GET_PICTURE_P1_SNAPSHOT_MODE, 0x00, 0x00, 0x00, 100);
  vTaskDelay(100 / portTICK_PERIOD_MS);

  // receive the DATA command

  uint8_t receive_buf[6];
  int     len = uart_read_bytes(UART_NUM_1, receive_buf, 6, 100 / portTICK_RATE_MS);

  if (receive_buf[0] == COMMAND_START && receive_buf[1] == DATA_ID && len == 6) {

    ret_val = (receive_buf[5] << 16) + (receive_buf[4] << 8) + receive_buf[3];

#ifdef DEBUG_OUTPUT
    printf("Incoming data size: %i \n", ret_val);
    printf("REC %i: \n", len);
    print_buffer_as_hex(receive_buf, len);
    printf("\ngot DATA, sending ACK\n");
    sendAck(DATA_ID);
    printf("\n---------------------------------\n");
#endif
  }

  return ret_val;
}

//}

/* receive_jpeg_image() //{ */

bool receive_jpeg_image(uint32_t image_size, uint16_t packet_size, uint8_t* dest_buffer) {

  bool     ret_val       = false;
  uint32_t bytes_written = 0;  // number of bytes written to the dest_buffer
  uint8_t  receive_buffer[packet_size];
  /* uint16_t expected_packet_id = 1; */  //  TODO: check the id of the incoming packets, also check the verification code

  while (1) {

    uint32_t got_bytes = uart_read_bytes(UART_NUM_1, receive_buffer, packet_size, 100 / portTICK_RATE_MS);

    if (got_bytes > 4) {
      uint16_t packet_data_size = (receive_buffer[3] << 8) + receive_buffer[2];

      if (bytes_written + packet_data_size > image_size) {
        printf("Received too much data, aborting!\n");
        ret_val = false;
        break;
      }

      for (int i = 0; i < packet_data_size; i++) {
        dest_buffer[bytes_written + i] = receive_buffer[i+4]; //first 4 bytes are not containing image data
      }
      bytes_written += packet_data_size;

      printf("Received %i/%i bytes\n", bytes_written, image_size);

      sendPacketAck(receive_buffer[1], receive_buffer[0]);
    }else{
        printf("Received invalid data, aborting!\n");
        ret_val = false;
        break;
    }

    if (bytes_written == image_size) {
      printf("Image data received succesfully\n");
      ret_val = true;
      break;
    }
    // TODO: add a timeout condition if no packets are arriving
  }

  return ret_val;
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

  uart_param_config(uart_num, &uart_config);
  uart_set_pin(uart_num, ECHO_TEST_TXD, ECHO_TEST_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, 0);

  uint8_t* datakeeper = (uint8_t*)malloc(38400 * sizeof(uint8_t));

  printf("running in ... 3\n");
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  printf("running in ... 2\n");
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  printf("running in ... 1\n");
  vTaskDelay(2000 / portTICK_PERIOD_MS);

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

  printf("sending package size 512 bytes\n");
  sendCommand(SET_PKG_SIZE_ID, SET_PKG_SIZE_P1, 0x00, 0x02, 0x09, 100);
  vTaskDelay(100 / portTICK_PERIOD_MS);

  printf("sending snapshot mode, jpeg\n");
  sendCommand(SNAPSHOT_ID, SNAPSHOT_P1_JPEG, 0x00, 0x00, 0x00, 100);
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  printf("requesting image\n");
  uint32_t image_size = request_image(GET_PICTURE_P1_SNAPSHOT_MODE);

  bool got_image = receive_jpeg_image(image_size, 512, datakeeper);  // packet size hardcoded for now

  vTaskDelay(2000 / portTICK_PERIOD_MS);
  if (got_image) {
    printf("sssss"); //synchronization message for the python image receiver
    print_buffer_as_hex(datakeeper, image_size);
    printf("sssss"); //synchronization message for the python image receiver
  }

  while (1) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    printf("we are done here");
  }
}

//}

