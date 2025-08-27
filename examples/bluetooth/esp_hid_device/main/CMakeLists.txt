set(srcs "esp_hid_device_main.c"
         "esp_hid_gap.c"
         "Kalman.cpp"
         "lsm6ds3.cpp")
set(include_dirs ".")

set(EXTRA_COMPONENT_DIRS ../components/)

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "${include_dirs}"
                       REQUIRES esp_hid
                       PRIV_REQUIRES nvs_flash driver I2Cdev LSM6DS3)
