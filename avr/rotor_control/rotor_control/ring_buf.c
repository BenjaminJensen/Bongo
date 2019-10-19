#include <avr/io.h>
#include "include/ring_buf.h"

unsigned int buf_getc(volatile struct ring_buf_s *buf)
{
	unsigned char tmptail;
	if(buf->head == buf->tail)
	{
		// Buffer empty
		return RING_BUF_EMPTY;
	}  

  /* calculate /store buffer index */
  tmptail = (((buf->tail) + 1) & buf->size);
  buf->tail = tmptail; 
  
  /* get data from receive buffer */
  return (int)(buf->data[tmptail] | 0x0100);
}

void buf_putc(volatile struct ring_buf_s *buf, unsigned char c)
{
	unsigned char tmphead;

  tmphead  = (buf->head + 1) & buf->size;
  
  while ( tmphead == buf->tail ){
      ;/* wait for free space in buffer */
  }
  
  buf->data[tmphead] = c;
  buf->head = tmphead;
}

void init_buf(volatile struct ring_buf_s *buf, unsigned char *data, unsigned char size)
{
	buf->head = buf->tail = 0;
	buf->data = data;
	buf->size = size;
}
