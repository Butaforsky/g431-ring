#ifndef __RING_BUFFER_H__
#define __RING_BUFFER_H__

/**
 *
 * @brief Ring buffer lib  *
 * @author butaforsky
 * @version 0.1.0
 */

#include "main.h"
/* User defines */
#define RING_BUF_LEN 256 
/* User enums */

/* User structs */
typedef struct
{
  volatile uint16_t tail;
  volatile uint16_t head;
  uint8_t buffer_data[RING_BUF_LEN];
  volatile uint16_t empty_slots;
  UART_HandleTypeDef *huart;

} rb_t;
/* User typedefs */

/* User variables */

/* User functions */
void ring_buff_put(rb_t *rb, uint8_t data);
int ring_buf_get_empty_slots(rb_t *rb);
int ring_buf_init(rb_t *rb, UART_HandleTypeDef *uart);
uint8_t ring_buf_get_head(rb_t *rb);

#endif
