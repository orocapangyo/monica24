#pragma once
#ifdef __cplusplus
extern "C" {
#endif
#include "stdint.h"
#include "stdbool.h"

#include "custom_transport.h"


rmw_ret_t set_microros_serial_transports(void);
rmw_ret_t set_microros_serial_transports_with_options(rmw_init_options_t * rmw_options);

#ifdef __cplusplus
}
#endif
