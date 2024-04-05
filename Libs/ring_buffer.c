/**
 *
 * @brief Ring buffer lib
 * @author butaforsky
 * @version 0.1.0
 */

#include "ring_buffer.h"
/* User defines */
/* User enums */

/* User structs */

/* User typedefs */

/* User variables */
extern UART_HandleTypeDef *huart2;

/* User functions */

void ring_buff_put(rb_t *rb, uint8_t data)
{
  if (rb->head != (RING_BUF_LEN - 1))
  {
    rb->buffer_data[rb->head] = data;
    rb->head++;
  }
  else
  {
    rb->head = 0;
    rb->buffer_data[rb->head] = data;
  }
}

int ring_buf_get_empty_slots(rb_t *rb)
{
  rb->empty_slots = ((uint16_t)(RING_BUF_LEN + rb->head - rb->tail)) % RING_BUF_LEN;
  return rb->empty_slots;
}

int ring_buf_init(rb_t *rb, UART_HandleTypeDef *uart)
{
  if (uart != NULL)
  {
    rb->huart = uart;
  }
  else{
    return 1;
  }
  rb->head = 0;
  rb->tail = 0;
  for (uint16_t i = 0; i < RING_BUF_LEN; i++)
  {
    rb->buffer_data[i] = 0;
  }
  rb->empty_slots = RING_BUF_LEN;
  return 0;
}

uint8_t ring_buf_get_head(rb_t *rb)
{
  uint8_t getc = 0;
  getc = rb->buffer_data[rb->tail];
  rb->buffer_data[rb->tail] = 0;
  if (rb->tail != (RING_BUF_LEN - 1))
  {
    rb->tail++;
  }
  else{
    rb->tail = 0;
  }
  return getc;
}
