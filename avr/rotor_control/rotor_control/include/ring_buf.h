#ifndef RING_BUF_H
#define RING_BUF_H

#define RING_BUF_EMPTY	0xFFFF

extern struct ring_buf_s {
	unsigned char *data;
	unsigned char head;
	unsigned char tail;
	unsigned char size;
};

extern unsigned int buf_getc(volatile struct ring_buf_s *buf);
extern void buf_putc(volatile struct ring_buf_s *buf, unsigned char c);
extern void init_buf(volatile struct ring_buf_s *buf, unsigned char *data, unsigned char size);


#endif