
#include "ucam_iii.h"

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

  sendAck(SYNC_ID);

#ifdef DEBUG_OUTPUT
  printf("Succesfully synced and sync id sent back, waiting 2000ms as per datasheet\n");
#endif

  vTaskDelay(2000 / portTICK_PERIOD_MS);
  return true;
}

//}

/* request_image() //{ */

uint32_t request_image(uint8_t image_type) {

  uint32_t ret_val = 0;

  sendCommand(GET_PICTURE_ID, image_type, 0x00, 0x00, 0x00, 100);

  // receive the DATA command

  uint8_t receive_buf[6];
  int     len = uart_read_bytes(UART_NUM_1, receive_buf, 6, 100 / portTICK_RATE_MS);

  if (receive_buf[0] == COMMAND_START && receive_buf[1] == DATA_ID && len == 6) {

    ret_val = (receive_buf[5] << 16) + (receive_buf[4] << 8) + receive_buf[3];
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
        dest_buffer[bytes_written + i] = receive_buffer[i + 4];  // first 4 bytes are not containing image data
      }
      bytes_written += packet_data_size;

      printf("Received %i/%i bytes\n", bytes_written, image_size);

      sendPacketAck(receive_buffer[1], receive_buffer[0]);
    } else {
      printf("Received invalid data %i, aborting!\n", got_bytes);
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

/* receive_raw_image() //{ */

bool receive_raw_image(uint16_t rec_buffer_size, uint8_t* dest_buffer) {

  bool     ret_val       = false;
  uint32_t bytes_written = 0;  // number of bytes written to the dest_buffer
  uint8_t  receive_buffer[rec_buffer_size];
  uint8_t  dummy_buffer[1];


  printf("requesting image\n");
  uint32_t image_size = request_image(GET_PICTURE_P1_SNAPSHOT_MODE);

  uint32_t read_data_bytes = 0;
  uint32_t dest_buffer_it  = 0;
  uint32_t iter_num        = 0;


  while (1) {

    uint32_t read_bytes_this_iter = 0;

    while (read_bytes_this_iter < read_data_bytes) {
      read_bytes_this_iter += uart_read_bytes(UART_NUM_1, dummy_buffer, 1, 1 / portTICK_RATE_MS);
    }

    uint32_t read_data_bytes_this_iter = uart_read_bytes(UART_NUM_1, receive_buffer, rec_buffer_size, 100 / portTICK_RATE_MS);
    read_data_bytes += read_data_bytes_this_iter;
    read_bytes_this_iter += read_data_bytes_this_iter;

    while (read_bytes_this_iter < image_size) {
      read_bytes_this_iter += uart_read_bytes(UART_NUM_1, dummy_buffer, 1, 1 / portTICK_RATE_MS);
    }

    for (int i = 0; i < read_data_bytes_this_iter; i++) {
      dest_buffer[dest_buffer_it + i] = receive_buffer[i];
      dest_buffer_it++;
    }

    printf("read bytes: %i/%i\n", read_data_bytes, image_size);

    sendAck(0x00);  // according to the datasheet, this ACK should not be necesary, but the camera freezes without it

    if (read_data_bytes == image_size) {
      printf("All data read!\n");
      ret_val = true;
      break;
    } else if (read_data_bytes > image_size) {
      printf("Too much data read!\n");
      break;
    }

    printf("End of loop number %i\n", iter_num);
    iter_num++;

    while (!request_image(GET_PICTURE_P1_SNAPSHOT_MODE)) {
      vTaskDelay(500 / portTICK_PERIOD_MS);
    }
  }

  return ret_val;
}

//}
