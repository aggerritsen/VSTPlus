idf_component_register(
    SRCS
        "main.c"
        "post_diagnostics.cpp"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        esp_system
        spi_flash
        esp_psram
        heap
        fatfs
        sdmmc
        driver
        vfs
        esp_timer
        esp_driver_uart
)
