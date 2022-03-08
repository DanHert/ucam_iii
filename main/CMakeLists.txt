idf_component_register(SRCS "main.c" "ucam-iii/src/ucam_iii.c" "ucam-iii/uart_glue/esp32_uart_glue.c"
                       INCLUDE_DIRS "ucam-iii/include" "ucam-iii/src" "ucam-iii/uart_glue"
                       REQUIRES mbedtls)
