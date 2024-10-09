set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)  # resulting in -std=gnu11

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

idf_component_register(SRCS "general_data_getter.cpp" "encrypt.c"
                       "sensors/bmp180/bmp180.c"
                       "sensors/dht11/dht11.c"
                       "sensors/yl69/yl69.c"
                       "sensors/acoustic/acoustic.c"
                       "sensors/RTC/rtc.c"
                       INCLUDE_DIRS "sensors/bmp180"
                                    "sensors/dht11"
                                    "sensors/yl69"
                                    "sensors/acoustic"
                                    "sensors/RTC"
                                    )

