file(GLOB_RECURSE COMPONENT_SRC *.c)

idf_component_register(
	SRCS ${COMPONENT_SRC}
    INCLUDE_DIRS "."
    REQUIRES driver inv_imu
    PRIV_REQUIRES i2c_master esp_timer
)

