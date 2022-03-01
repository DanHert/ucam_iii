idf_component_register(SRCS "main.c" "ucam-iii/src/ucam_iii.c"
                       INCLUDE_DIRS "ucam-iii/include" "ucam-iii/src"
                       REQUIRES mbedtls)
