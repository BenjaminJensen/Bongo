#ifndef TCP_LOGGER_H_
#define TCP_LOGGER_H_

void tcp_logger_init(void);
int _log_vprintf(const char *fmt, va_list args);

#endif