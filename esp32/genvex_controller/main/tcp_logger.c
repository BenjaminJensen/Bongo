#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "lwip/err.h"
#include "lwip/sockets.h"

//-----------------------------
// Declarations
//-----------------------------
#define PORT 1234

#define BUF_SIZE (128) 
#define QUEUE_SIZE (128) 
static const char *TAG = "TCP Logger";

static uint8_t logs_data[BUF_SIZE * QUEUE_SIZE]; // 16kb

/* The variable used to hold the queue's data structure. */
static StaticQueue_t xStaticQueue;

QueueHandle_t log_queue;

// TCP client task 
static void tcp_client_task(void *pvParameters);

//-----------------------------
// Public functions
//-----------------------------
void tcp_logger_init(void)
{

    /* Create a queue capable of containing 10 uint64_t values. */
    log_queue = xQueueCreateStatic( QUEUE_SIZE,
                                 BUF_SIZE,
                                 logs_data,
                                 &xStaticQueue );

    /* pxQueueBuffer was not NULL so xQueue should not be NULL. */
    configASSERT( log_queue );

    xTaskCreate(tcp_client_task, "tcp_client", 4096, NULL, 5, NULL);
}

int _log_vprintf(const char *fmt, va_list args) {
    char buf[256];
    char tmp[BUF_SIZE];
    int len;

    len = vsprintf(buf, fmt, args);

    if(len >= 256) {
        printf("ERROR! Buffer overflow in _log_vprintf\n");
    }
    else {
        buf[len] = 0; // Terminate string
        if(uxQueueSpacesAvailable( log_queue ) == 0) {
            // Queue is full remove oldest item
            xQueueReceive(log_queue, tmp, 10);
        }
        
        // Post to queue
        if( xQueueSendToBack(log_queue, buf, 10 / portTICK_PERIOD_MS) == errQUEUE_FULL ) {
            // post failed
            printf("ERROR: failed to send to queue in _log_vprintf");
        }
    
    }

    // #3 ALWAYS Write to stdout!
    return vprintf(fmt, args);
}

//-----------------------------
// Private functions
//-----------------------------

static void tcp_client_task(void *pvParameters)
{
    char buffer[128];
    char host_ip[] = "192.168.1.252";
    int addr_family = 0;
    int ip_protocol = 0;

    while (1) {
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(host_ip);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;

        int sock =  socket(addr_family, SOCK_STREAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        }
        ESP_LOGI(TAG, "Socket created, connecting to %s:%d", host_ip, PORT);

        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(struct sockaddr_in6));
        if (err != 0) {
            ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
        }

        ESP_LOGI(TAG, "Successfully connected");
        
        int send_in_cycle;
        while (1) {
            send_in_cycle = 0;
            printf("messages in queue: %d", uxQueueMessagesWaiting(log_queue));
            while(xQueueReceive(log_queue, buffer, 10) ) {
                int err = send(sock, buffer, strlen(buffer), 0);
                if (err < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    break;
                }
                else {
                    send_in_cycle = 1;
                }
            }

            if(send_in_cycle == 0)
            {
                // send alive
                int len = sprintf(buffer, "ACK");
                buffer[len] = 0;
                int err = send(sock, buffer, strlen(buffer), 0);
                if (err < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending ACK: errno %d", errno);
                    break;
                }
            }
            if (err < 0) {
                ESP_LOGE(TAG, "Outer loop Error occurred during sending: errno %d", errno);
                break;
            }
           
            vTaskDelay(10000 / portTICK_PERIOD_MS);
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

