/*
 * aws_client.c
 *
 *  Created on: 14. feb. 2018
 *      Author: Benjamin
 */
/************************************************
 * Includes
 ***********************************************/

#include "aws_client.h"
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>

#include <freertos/task.h>
#include <esp_system.h>
#include "esp_event_loop.h"
#include "esp_log.h"

#include "aws_iot_config.h"
#include "aws_iot_log.h"
#include "aws_iot_version.h"


/************************************************
 * Defines
 ***********************************************/

#define CONFIG_AWS_EXAMPLE_CLIENT_ID "Genvex"
#define NUM_PUBS	8

/************************************************
 * Locals
 ***********************************************/
/**
 * @brief Default MQTT HOST URL is pulled from the aws_iot_config.h
 */
//char HostAddress[255] = "192.168.1.140";
char HostAddress[255] = "192.168.1.252";

/**
 * @brief Default MQTT port is pulled from the aws_iot_config.h
 */
uint32_t port = 8883;

static
AWS_IoT_Client client;

static const char *TAG = "AWS_CLI";

extern const uint8_t aws_root_ca_pem_start[] asm("_binary_aws_root_ca_pem_start");
extern const uint8_t aws_root_ca_pem_end[] asm("_binary_aws_root_ca_pem_end");
extern const uint8_t certificate_pem_crt_start[] asm("_binary_certificate_pem_crt_start");
extern const uint8_t certificate_pem_crt_end[] asm("_binary_certificate_pem_crt_end");
extern const uint8_t private_pem_key_start[] asm("_binary_private_pem_key_start");
extern const uint8_t private_pem_key_end[] asm("_binary_private_pem_key_end");

extern void handle_set_speed(AWS_IoT_Client *pClient, char *topicName, uint16_t topicNameLen,
        IoT_Publish_Message_Params *params, void *pData);
//static TaskHandle_t aws_iot_task;

static EventGroupHandle_t *wifi_event_group;
static EventBits_t CONNECTED_BIT;

// Storage for async publishers
static aws_publish_t pubs[NUM_PUBS];
static EventGroupHandle_t publisherReady;
static uint8_t numPublishClients;
SemaphoreHandle_t pubMutex;
/************************************************
 * Local function declarations
 ***********************************************/
static void aws_iot_task(void *param);

/************************************************
 * Exported functions
 ***********************************************/

void init_aws_client(EventGroupHandle_t *weg, EventBits_t bit) {
	wifi_event_group = weg;
	CONNECTED_BIT = bit;

	// Setup publish client interface
	numPublishClients = 0;
	publisherReady = xEventGroupCreate();
	pubMutex = xSemaphoreCreateMutex();

	// Create AWS task
	xTaskCreatePinnedToCore(&aws_iot_task, "aws_iot_task", 36*1024, NULL, 15, NULL, 1);
}

IoT_Error_t aws_client_sub(const char *topic, pApplicationHandler_t handler, QoS q) {
	IoT_Error_t rc = FAILURE;
	rc = aws_iot_mqtt_subscribe(&client, topic, strlen(topic), q, handler, NULL);
	return rc;
}

aws_publish_t* aws_reqister_publish_client(const char *pTopicName, uint16_t topicNameLen, IoT_Publish_Message_Params *pParams) {
	aws_publish_t* ret = NULL;
	if(xSemaphoreTake( pubMutex, ( TickType_t ) 10 ) == pdTRUE) {
		// There must be available slots
		if(numPublishClients < (NUM_PUBS - 1)) {
			ret = &(pubs[numPublishClients]);
			ret->eventBit = 1 << numPublishClients;
			ret->mutex = xSemaphoreCreateMutex();
			ret->pTopicName = pTopicName;
			ret->topicNameLen = topicNameLen;
			ret->parms = pParams;

			// Incrtement number of active client slots
			numPublishClients++;
		}
		xSemaphoreGive( pubMutex );
	}
	return ret;
}

IoT_Error_t aws_client_pub(aws_publish_t* pub, void* payload, int payloadLen) {
	IoT_Error_t rc = FAILURE;

	if(xSemaphoreTake( pub->mutex, ( TickType_t ) 10 ) == pdTRUE) {
		pub->parms->payloadLen = payloadLen;
		pub->parms->payload = payload;
		// Signal slot is ready to publish
		xEventGroupSetBits(publisherReady, pub->eventBit);
		xSemaphoreGive( pub->mutex );
		rc = SUCCESS;
	}
	return rc;
}

/************************************************
 * Local functions
 ***********************************************/
void disconnectCallbackHandler(AWS_IoT_Client *pClient, void *data) {
    ESP_LOGW(TAG, "MQTT Disconnect");
    IoT_Error_t rc = FAILURE;

    if(NULL == pClient) {
        return;
    }

    if(aws_iot_is_autoreconnect_enabled(pClient)) {
        ESP_LOGI(TAG, "Auto Reconnect is enabled, Reconnecting attempt will start now");
    } else {
        ESP_LOGW(TAG, "Auto Reconnect not enabled. Starting manual reconnect...");
        rc = aws_iot_mqtt_attempt_reconnect(pClient);
        if(NETWORK_RECONNECTED == rc) {
            ESP_LOGW(TAG, "Manual Reconnect Successful");
        } else {
            ESP_LOGW(TAG, "Manual Reconnect Failed - %d", rc);
        }
    }
}
void iot_subscribe_callback_handler(AWS_IoT_Client *pClient, char *topicName, uint16_t topicNameLen,
                                    IoT_Publish_Message_Params *params, void *pData) {
    ESP_LOGI(TAG, "Subscribe callback");
    ESP_LOGI(TAG, "%.*s\t%.*s", topicNameLen, topicName, (int) params->payloadLen, (char *)params->payload);
}
static void aws_iot_task(void *param) {
    IoT_Error_t rc = FAILURE;
    int i;
    IoT_Client_Init_Params mqttInitParams = iotClientInitParamsDefault;
    IoT_Client_Connect_Params connectParams = iotClientConnectParamsDefault;

    ESP_LOGI(TAG, "AWS IoT SDK Version %d.%d.%d-%s", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, VERSION_TAG);

    mqttInitParams.enableAutoReconnect = false; // We enable this later below
    mqttInitParams.pHostURL = HostAddress;
    mqttInitParams.port = port;

    mqttInitParams.pRootCALocation = (const char *)aws_root_ca_pem_start;
    mqttInitParams.pDeviceCertLocation = (const char *)certificate_pem_crt_start;
    mqttInitParams.pDevicePrivateKeyLocation = (const char *)private_pem_key_start;


    mqttInitParams.mqttCommandTimeout_ms = 20000;
    mqttInitParams.tlsHandshakeTimeout_ms = 5000;
    mqttInitParams.isSSLHostnameVerify = true;
    mqttInitParams.disconnectHandler = disconnectCallbackHandler;
    mqttInitParams.disconnectHandlerData = NULL;

    rc = aws_iot_mqtt_init(&client, &mqttInitParams);
    if(SUCCESS != rc) {
        ESP_LOGE(TAG, "aws_iot_mqtt_init returned error : %d ", rc);
        abort();
    }

    /* Wait for WiFI to show as connected */
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
                        false, true, portMAX_DELAY);

    connectParams.keepAliveIntervalInSec = 10;
    connectParams.isCleanSession = true;
    connectParams.MQTTVersion = MQTT_3_1_1;
    /* Client ID is set in the menuconfig of the example */
    connectParams.pClientID = CONFIG_AWS_EXAMPLE_CLIENT_ID;
    connectParams.clientIDLen = (uint16_t) strlen(CONFIG_AWS_EXAMPLE_CLIENT_ID);
    connectParams.isWillMsgPresent = false;

    ESP_LOGI(TAG, "Connecting to AWS...");
    do {
        rc = aws_iot_mqtt_connect(&client, &connectParams);
        if(SUCCESS != rc) {
            ESP_LOGE(TAG, "Error(%d) connecting to %s:%d", rc, mqttInitParams.pHostURL, mqttInitParams.port);
            vTaskDelay(1000 / portTICK_RATE_MS);
        }
    } while(SUCCESS != rc);

    /*
     * Enable Auto Reconnect functionality. Minimum and Maximum time of Exponential backoff are set in aws_iot_config.h
     *  #AWS_IOT_MQTT_MIN_RECONNECT_WAIT_INTERVAL
     *  #AWS_IOT_MQTT_MAX_RECONNECT_WAIT_INTERVAL
     */
    rc = aws_iot_mqtt_autoreconnect_set_status(&client, true);
    if(SUCCESS != rc) {
        ESP_LOGE(TAG, "Unable to set Auto Reconnect to true - %d", rc);
        abort();
    }

    const char *TOPIC = "genvex/set_speed";
	const int TOPIC_LEN = strlen(TOPIC);
    ESP_LOGI(TAG, "Subscribing...");
	rc = aws_iot_mqtt_subscribe(&client, TOPIC, TOPIC_LEN, QOS0, handle_set_speed, NULL);
	if(SUCCESS != rc) {
		ESP_LOGE(TAG, "Error subscribing : %d ", rc);
		abort();
	}

    while((NETWORK_ATTEMPTING_RECONNECT == rc || NETWORK_RECONNECTED == rc || SUCCESS == rc)) {

        //Max time the yield function will wait for read messages
        rc = aws_iot_mqtt_yield(&client, 200);
        if(NETWORK_ATTEMPTING_RECONNECT == rc) {
            // If the client is attempting to reconnect we will skip the rest of the loop.
            continue;
        }

        for(i = 0; i < numPublishClients;i++) {
        	if(xEventGroupGetBits(publisherReady) & pubs[i].eventBit) {
        		 /* See if we can obtain the semaphore.  If the semaphore is not
				available wait 10 ticks to see if it becomes free. */
				if( xSemaphoreTake( pubs[i].mutex, ( TickType_t ) 10 ) == pdTRUE )
				{
					/* We were able to obtain the semaphore and can now access the
					shared resource. */

					xEventGroupClearBits(publisherReady, pubs[i].eventBit);
					rc = aws_iot_mqtt_publish(&client, pubs[i].pTopicName, pubs[i].topicNameLen, pubs[i].parms);
					if(rc != SUCCESS) {
						ESP_LOGE(TAG, "Error(%d) publishing : %s ", rc, pubs[i].pTopicName);
					}

					/* We have finished accessing the shared resource.  Release the
					semaphore. */
					xSemaphoreGive( pubs[i].mutex );
				}
				else
				{
					/* We could not obtain the semaphore and can therefore not access
					the shared resource safely. */
					ESP_LOGE(TAG, "Erroroptaning publishing mutex : %d ", pubs[i].eventBit);
				}

        	}
        }

        vTaskDelay(500 / portTICK_RATE_MS);
    }

    ESP_LOGE(TAG, "An error occurred in the main loop. %d", rc);
    abort();
}
