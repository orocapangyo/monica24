file(GLOB_RECURSE COMPONENT_SRC *.c)

idf_component_register(
	SRCS ${COMPONENT_SRC}
    INCLUDE_DIRS "."
    REQUIRES driver nvs_flash esp_wifi
)
