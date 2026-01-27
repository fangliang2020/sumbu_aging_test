set(srcs    "main.c"
            "bsp/drivers/sys_twai.c"
            "app/src/can_comm.c"
            )

set(INCLUDE_DIRS "." 
            "app/inc" 
            "bsp/drivers"
            )

# 添加板级公共文件
# file(GLOB BOARD_COMMON_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/common/*.c)
# list(APPEND srcs ${BOARD_COMMON_SOURCES})
# list(APPEND INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/common/)
## list(APPEND 用于向列表变量中添加元素。

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS ${INCLUDE_DIRS}
                       REQUIRES esp_driver_uart esp_driver_gpio esp_driver_i2c esp_driver_twai bt nvs_flash spi_flash esp_driver_rmt esp_driver_ledc
                       PRIV_REQUIRES  driver esp_wifi ulp  esp_pm esp_driver_gptimer
                    #    WHOLE_ARCHIVE
                       )
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Os
    -fdata-sections
    -ffunction-sections
    -Wl,--gc-sections
)      
add_compile_options(-Werror=null-dereference)
add_compile_options(-Wno-unused-const-variable)
