# Register your main source file as a component
idf_component_register(SRCS "general_data_getter.cpp"
                       INCLUDE_DIRS "." 
                                    "sensors/bmp180" 
                                    "sensors/dht22" 
                                    "sensors/yl69"
				    "sensors/acoustic")
                                    
target_compile_options(${COMPONENT_LIB} PRIVATE -fexceptions)
