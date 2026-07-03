#ifndef CHASSIS_PROTOCOL_H
#define CHASSIS_PROTOCOL_H

#include "chassis_config.h"
#include "usart.h"

#ifdef __cplusplus
extern "C" {
#endif

void chassis_protocol_init(UART_HandleTypeDef *huart);
void chassis_protocol_restart_rx(void);
void chassis_protocol_set_mode(chassis_protocol_mode_t mode);
chassis_protocol_mode_t chassis_protocol_get_mode(void);
void chassis_protocol_enqueue_rx_byte_from_isr(uint8_t byte);
bool chassis_protocol_take_rx_byte(uint8_t *out_byte);
void chassis_protocol_on_rx_byte(uint8_t byte);
bool chassis_protocol_poll(chassis_cmd_t *out_cmd);
void chassis_protocol_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_PROTOCOL_H */
