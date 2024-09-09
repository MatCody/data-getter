idf_component_register(SRCS "general_data_getter.cpp"
                       INCLUDE_DIRS "." 
                                    "sensors/bmp180" 
                                    "sensors/dht22" 
                                    "sensors/yl69")

# Add the following lines to register your sensor components
#idf_component_register(SRCS "sensors/bmp180/bmp180.c"
#                       INCLUDE_DIRS "sensors/bmp180")
#
#idf_component_register(SRCS "sensors/dht22/dht22.c"
#                       INCLUDE_DIRS "sensors/dht22")
#
# If you're actually using the YL69 sensor, uncomment the following line
# idf_component_register(SRCS "sensors/yl69/yl69.c"
#                        INCLUDE_DIRS "sensors/yl69")
