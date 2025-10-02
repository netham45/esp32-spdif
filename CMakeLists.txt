idf_component_register( SRCS "histogram.c" "spdif_in.c"
                        INCLUDE_DIRS "include"
                        PRIV_REQUIRES esp_ringbuf esp_driver_rmt)
