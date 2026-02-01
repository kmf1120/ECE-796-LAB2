# CMake for LSM6DSV16X ST driver component
idf_component_register(SRCS "lsm6dsv16x_reg.c"
                       INCLUDE_DIRS ""
                       PRIV_REQUIRES driver)
