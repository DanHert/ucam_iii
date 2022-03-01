
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
#include "ucam_iii.h"


#define ECHO_TEST_TXD (17)
#define ECHO_TEST_RXD (16)
#define BUF_SIZE (1024)

#ifdef CONFIG_IDF_TARGET_ESP32
#define CHIP_NAME "ESP32"
#endif

#ifdef CONFIG_IDF_TARGET_ESP32S2BETA
#define CHIP_NAME "ESP32-S2 Beta"
#endif

/* app_main() //{ */

void app_main(void) {

  const int     uart_num    = UART_NUM_1;
  uart_config_t uart_config = {
      .baud_rate           = 921600,
      /* .baud_rate           = 115200, */
      .data_bits           = UART_DATA_8_BITS,
      .parity              = UART_PARITY_DISABLE,
      .stop_bits           = UART_STOP_BITS_1,
      .flow_ctrl           = UART_HW_FLOWCTRL_DISABLE,
      .rx_flow_ctrl_thresh = 122,
  };

  uart_param_config(uart_num, &uart_config);
  uart_set_pin(uart_num, ECHO_TEST_TXD, ECHO_TEST_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, 0);

  uint8_t* datakeeper = (uint8_t*)malloc(38400 * sizeof(uint8_t)); // simulation of image storage

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
  /* sendCommand(INIT_ID, 0x00, INIT_P2_JPEG, 0x07, INIT_P4_JPEG_320X240, 100); */
  /* sendCommand(INIT_ID, 0x00, INIT_P2_JPEG, 0x07, INIT_P4_JPEG_640X480, 100); */
  sendCommand(INIT_ID, 0x00, INIT_P2_JPEG, 0x07, INIT_P4_JPEG_160X128, 100);
  vTaskDelay(100 / portTICK_PERIOD_MS);

  printf("sending package size 512 bytes\n");
  sendCommand(SET_PKG_SIZE_ID, SET_PKG_SIZE_P1, 0x00, 0x02, 0x09, 100);
  vTaskDelay(100 / portTICK_PERIOD_MS);

  printf("sending snapshot mode, jpeg\n");
  sendCommand(SNAPSHOT_ID, SNAPSHOT_P1_JPEG, 0x00, 0x00, 0x00, 100);
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  printf("requesting image\n");
  uint32_t image_size = request_image(GET_PICTURE_P1_SNAPSHOT_MODE);

  printf("requesting image compl %i\n" , image_size);
  bool got_image = receive_jpeg_image(image_size, 512, datakeeper);  // packet size hardcoded for now

  vTaskDelay(2000 / portTICK_PERIOD_MS);
  if (got_image) {
    printf("sssss"); //synchronization message for the python image receiver
    print_buffer_as_hex(datakeeper, image_size);
    printf("sssss"); //synchronization message for the python image receiver
    fflush(stdout);
  }


  /* printf("sending init\n"); */
  /* sendCommand(INIT_ID, 0x00, INIT_P2_RAW_8BIT_GRAYSCALE, INIT_P3_RAW_160X120, INIT_P4_JPEG_640X480, 100); */
  /* vTaskDelay(100 / portTICK_PERIOD_MS); */

  /* printf("sending snapshot mode, raw\n"); */
  /* sendCommand(SNAPSHOT_ID, SNAPSHOT_P1_RAW, 0x00, 0x00, 0x00, 100); */
  /* vTaskDelay(1000 / portTICK_PERIOD_MS); */

  /* bool got_image = receive_raw_image(1024, datakeeper);  // packet size hardcoded for now */

  /* int     len = uart_read_bytes(UART_NUM_1, datakeeper, 40000, 1000 / portTICK_RATE_MS); */
  /* if (got_image) { */
  /*   printf("sssss"); //synchronization message for the python image receiver */
  /*   print_buffer_as_hex(datakeeper, 19200); */
  /*   printf("sssss"); //synchronization message for the python image receiver */
  /*   fflush(stdout); */
  /* } */

  /* while (1) { */
  /*   vTaskDelay(1000 / portTICK_PERIOD_MS); */
  /*   printf("done"); */
  /* } */
}

//}

