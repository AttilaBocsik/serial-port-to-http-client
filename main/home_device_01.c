/**
 * @file barometer_sensor_http_client.c
 * @author Attila Bocsik (a.bocsik@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2022-01-29
 * 
 * @copyright Copyright (c) 2022
 * 
 */
/*
Schematic:
       PIC16F877A               NodeMCU ESP32
              VCC        <==>        5V 
              GND        <==>        GND
              TX         <==>        P09(GPIO16) RX2 
              RX         <==>        P10(GPIO17) TX2
       Other 
              VCC        <==>        5V 
              GND        <==>        GND
              TX         <==>        P09(GPIO9)  RX1
              RX         <==>        P10(GPIO10) TX1

       LED(RED_LED)(+)   <==>        GPIO5
       LED(GREEN_LED)(+) <==>        GPIO4
       LED(YELLOW_LED)(+) <==>       GPIO18
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_types.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_eth.h"
#include "esp_wifi.h"
#include "esp_tls.h"
#include "nvs_flash.h"
#include "esp_http_client.h"
#include "sdkconfig.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#define RED_LED 5
#define GREEN_LED 4
#define YELLOW_LED 18

static const int RX_BUF_SIZE = 1024;
#define TXD_PIN_2 (GPIO_NUM_17)
#define RXD_PIN_2 (GPIO_NUM_16)
#define TXD_PIN_1 (GPIO_NUM_10)
#define RXD_PIN_1 (GPIO_NUM_9)

#define MAX_HTTP_RECV_BUFFER 512
#define MAX_HTTP_OUTPUT_BUFFER 2048

#define systime_ms() (xTaskGetTickCount() * portTICK_PERIOD_MS)
#define delay_ms(t) vTaskDelay((t) / portTICK_PERIOD_MS)

/* WIFI SSID
 * WIFI password
 * WIFI connect maximum retry
 */
#define PROJECT_ESP_WIFI_SSID CONFIG_ESP_WIFI_SSID
#define PROJECT_ESP_WIFI_PASS CONFIG_ESP_WIFI_PASSWORD
#define PROJECT_ESP_MAXIMUM_RETRY CONFIG_ESP_MAXIMUM_RETRY

/* FreeRTOS event group to signal when we are connected */
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static const char *INFO = "[INFO]";
static const char *RX_TASK_TAG = "[RX_TASK]";
//unsigned int _counter = 0;
bool isConnectedWifi = false;

/**
 * Attribute in message data
 */
typedef struct
{
       char *uuid;
       float latitude;
       float longitude;
       char *value;
       char *denomination;
       char *message;
       char *error;
} telegraph_t;

/**
 *@name LED initialize
 */
static void led_init()
{
       gpio_pad_select_gpio(RED_LED);
       gpio_pad_select_gpio(GREEN_LED);
       gpio_pad_select_gpio(YELLOW_LED);
       gpio_set_direction(RED_LED, GPIO_MODE_OUTPUT);
       gpio_set_direction(GREEN_LED, GPIO_MODE_OUTPUT);
       gpio_set_direction(YELLOW_LED, GPIO_MODE_OUTPUT);
       gpio_set_level(RED_LED, 0);
       gpio_set_level(GREEN_LED, 0);
       gpio_set_level(YELLOW_LED, 0);
}

static void echo_send(const int port, const char *str, uint8_t length)
{
       if (uart_write_bytes(port, str, length) != length)
       {
              ESP_LOGE(RX_TASK_TAG, "Send data critical failure.");
              // add your code to handle sending failure here
              abort();
       }
}

/**
 *@name UART 1 initialize
 */
void initUart1(void)
{
       const uart_config_t uart_config = {
           .baud_rate = 9600,
           .data_bits = UART_DATA_8_BITS,
           .parity = UART_PARITY_DISABLE,
           .stop_bits = UART_STOP_BITS_1,
           .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
       ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
       ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, TXD_PIN_1, RXD_PIN_1, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
       // We won't use a buffer for sending data.
       ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0));
}

/**
 *@name UART 2 initialize
 */
void initUart2(void)
{
       const uart_config_t uart_config = {
           .baud_rate = 9600,
           .data_bits = UART_DATA_8_BITS,
           .parity = UART_PARITY_DISABLE,
           .stop_bits = UART_STOP_BITS_1,
           .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
       ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart_config));
       ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, TXD_PIN_2, RXD_PIN_2, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
       // We won't use a buffer for sending data.
       ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, RX_BUF_SIZE * 2, 0, 0, NULL, 0));
}

/**
 * @brief Event handler method
 * 
 * @param arg 
 * @param event_base 
 * @param event_id 
 * @param event_data 
 * @return void
 */
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
       if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
       {
              esp_wifi_connect();
              ESP_LOGI(INFO, "WIFI_EVENT_STA_START");
              gpio_set_level(RED_LED, 1);
              gpio_set_level(GREEN_LED, 0);
              isConnectedWifi = false;
       }
       else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
       {
              esp_wifi_connect();
              ESP_LOGI(INFO, "WIFI_EVENT_STA_DISCONNECTED");
              gpio_set_level(RED_LED, 1);
              gpio_set_level(GREEN_LED, 0);
              isConnectedWifi = false;
       }
       else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
       {
              ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
              ESP_LOGI(INFO, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
              ESP_LOGI(INFO, "Connection WI-FI");
              gpio_set_level(RED_LED, 0);
              gpio_set_level(GREEN_LED, 1);
              isConnectedWifi = true;
       }
}

/**
 * @brief WIFI Initialization method
 * 
 */
void wifi_init_sta(void)
{
       s_wifi_event_group = xEventGroupCreate();
       ESP_ERROR_CHECK(esp_netif_init());
       ESP_ERROR_CHECK(esp_event_loop_create_default());
       esp_netif_create_default_wifi_sta();
       wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
       ESP_ERROR_CHECK(esp_wifi_init(&cfg));
       esp_event_handler_instance_t instance_any_id;
       esp_event_handler_instance_t instance_got_ip;
       ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                           ESP_EVENT_ANY_ID,
                                                           &event_handler,
                                                           NULL,
                                                           &instance_any_id));
       ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                           IP_EVENT_STA_GOT_IP,
                                                           &event_handler,
                                                           NULL,
                                                           &instance_got_ip));

       wifi_config_t wifi_config = {
           .sta = {
               .ssid = PROJECT_ESP_WIFI_SSID,
               .password = PROJECT_ESP_WIFI_PASS,
               /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
               .threshold.authmode = WIFI_AUTH_WPA2_PSK,

               .pmf_cfg = {
                   .capable = true,
                   .required = false},
           },
       };
       ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
       ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
       /* esp_wifi_start() method
       @return
       ESP_OK: succeed
       ESP_ERR_WIFI_NOT_INIT: WiFi is not initialized by esp_wifi_init
       ESP_ERR_INVALID_ARG: invalid argument
       ESP_ERR_NO_MEM: out of memory
       ESP_ERR_WIFI_CONN: WiFi internal error, station or soft-AP control block wrong
       ESP_FAIL: other WiFi internal errors
       */
       esp_err_t ret_esp_wifi_start;
       ret_esp_wifi_start = esp_wifi_start();
       switch (ret_esp_wifi_start)
       {
       case ESP_OK:
              ESP_LOGI(INFO, "esp_wifi_start: succeed");
              break;
       case ESP_ERR_WIFI_NOT_INIT:
              ESP_LOGI(INFO, "esp_wifi_start: WiFi is not initialized by esp_wifi_init");
              break;
       case ESP_ERR_INVALID_ARG:
              ESP_LOGI(INFO, "esp_wifi_start: invalid argument");
              break;
       case ESP_ERR_NO_MEM:
              ESP_LOGI(INFO, "esp_wifi_start: out of memory");
              break;
       case ESP_ERR_WIFI_CONN:
              ESP_LOGI(INFO, "esp_wifi_start: WiFi internal error, station or soft-AP control block wrong");
              break;
       case ESP_FAIL:
              ESP_LOGI(INFO, "esp_wifi_start: other WiFi internal errors");
              break;

       default:
              break;
       }
       ESP_LOGI(INFO, "wifi_init_sta finished.");
}

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
       static char *output_buffer; // Buffer to store response of http request from event handler
       static int output_len;      // Stores number of bytes read
       switch (evt->event_id)
       {
       case HTTP_EVENT_ERROR:
              ESP_LOGD(INFO, "HTTP_EVENT_ERROR");
              break;
       case HTTP_EVENT_ON_CONNECTED:
              ESP_LOGD(INFO, "HTTP_EVENT_ON_CONNECTED");
              break;
       case HTTP_EVENT_HEADER_SENT:
              ESP_LOGD(INFO, "HTTP_EVENT_HEADER_SENT");
              break;
       case HTTP_EVENT_ON_HEADER:
              ESP_LOGD(INFO, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
              break;
       case HTTP_EVENT_ON_DATA:
              ESP_LOGD(INFO, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
              /*
             *  Check for chunked encoding is added as the URL for chunked encoding used in this example returns binary data.
             *  However, event handler can also be used in case chunked encoding is used.
             */
              if (!esp_http_client_is_chunked_response(evt->client))
              {
                     // If user_data buffer is configured, copy the response into the buffer
                     if (evt->user_data)
                     {
                            memcpy(evt->user_data + output_len, evt->data, evt->data_len);
                     }
                     else
                     {
                            if (output_buffer == NULL)
                            {
                                   output_buffer = (char *)malloc(esp_http_client_get_content_length(evt->client));
                                   output_len = 0;
                                   if (output_buffer == NULL)
                                   {
                                          ESP_LOGE(INFO, "Failed to allocate memory for output buffer");
                                          return ESP_FAIL;
                                   }
                            }
                            memcpy(output_buffer + output_len, evt->data, evt->data_len);
                     }
                     output_len += evt->data_len;
              }

              break;
       case HTTP_EVENT_ON_FINISH:
              ESP_LOGD(INFO, "HTTP_EVENT_ON_FINISH");
              if (output_buffer != NULL)
              {
                     // Response is accumulated in output_buffer. Uncomment the below line to print the accumulated response
                     // ESP_LOG_BUFFER_HEX(TAG, output_buffer, output_len);
                     free(output_buffer);
                     output_buffer = NULL;
                     output_len = 0;
              }
              break;
       case HTTP_EVENT_DISCONNECTED:
              ESP_LOGI(INFO, "HTTP_EVENT_DISCONNECTED");
              //int mbedtls_err = 0;
              /*
            esp_err_t err = esp_tls_get_and_clear_last_error(evt->data, &mbedtls_err, NULL);
            if (err != 0) {
                if (output_buffer != NULL) {
                    free(output_buffer);
                    output_buffer = NULL;
                    output_len = 0;
                }
                ESP_LOGI(HTTP, "Last esp error code: 0x%x", err);
                ESP_LOGI(HTTP, "Last mbedtls failure: 0x%x", mbedtls_err);
            }
            */
              break;
       }
       return ESP_OK;
}

static void send_telegraph_http(const unsigned char *input)
{
       char data[22] = {0};
       char *dataStr = malloc(22);
       sprintf(data, "%s", input);
       strcpy(dataStr, data);
       printf("input %s\n", dataStr);
       char *buffer = malloc(MAX_HTTP_RECV_BUFFER + 1);
       if (buffer == NULL)
       {
              ESP_LOGE(INFO, "Cannot malloc http receive buffer");
              return;
       }

       esp_http_client_config_t config = {
           .url = "https://device-backend-api.herokuapp.com/device",
           .event_handler = _http_event_handler,
           .username = "attis71",
           .password = "Za1957",
           .auth_type = HTTP_AUTH_TYPE_BASIC};
       esp_http_client_handle_t client = esp_http_client_init(&config);
       esp_err_t err;

       char telegraph[218] = {0};
       stpcpy(telegraph, "uuid=f86e31f0-c924-40f1-83cf-763b8b3940df");
       strcat(telegraph, "&latitude=46.409636");
       strcat(telegraph, "&longitude=20.743210");
       strcat(telegraph, "&value=");
       strcat(telegraph, dataStr);
       strcat(telegraph, "&denomination=Sensor");
       strcat(telegraph, "&createDate=not");
       strcat(telegraph, "&message=Home data");
       strcat(telegraph, "&error=No");
       //const char *post_data = telegraph;
       char *post_data = malloc(217);
       stpcpy(post_data, telegraph);
       printf("post_data: %s\n", post_data);

       esp_http_client_set_method(client, HTTP_METHOD_POST);
       //esp_http_client_set_header(client, "Content-Type", "application/x-www-form-urlencoded");
       esp_http_client_set_post_field(client, post_data, strlen(post_data));
       while (1)
       {
              err = esp_http_client_perform(client);
              if (err != ESP_ERR_HTTP_EAGAIN)
              {
                     break;
              }
       }
       int content_length = esp_http_client_fetch_headers(client);
       int total_read_len = 0, read_len;
       if (total_read_len < content_length && content_length <= MAX_HTTP_RECV_BUFFER)
       {
              read_len = esp_http_client_read(client, buffer, content_length);
              if (read_len <= 0)
              {
                     ESP_LOGE(INFO, "Error read data");
              }
              buffer[read_len] = 0;
              ESP_LOGD(INFO, "read_len = %d", read_len);
       }
       ESP_LOGI(INFO, "HTTP Stream reader Status = %d, content_length = %d",
                esp_http_client_get_status_code(client),
                esp_http_client_get_content_length(client));

       printf("%s", buffer);

       esp_http_client_close(client);
       esp_http_client_cleanup(client);
       free(buffer);
       free(post_data);
       free(dataStr);
       /*
    if (err == ESP_OK) {
        ESP_LOGI(HTTP, "HTTP Status = %d, content length = %d",
                esp_http_client_get_status_code(client),
                esp_http_client_get_content_length(client));
    } else {
        ESP_LOGE(HTTP, "Error perform http request %s", esp_err_to_name(err));
    }
    esp_http_client_cleanup(client);
    */
}

/**
 * UARt RX function
 * @brief 
 * @name rx_task
 * @param pvParameters 
 */
static void rx_task(void)
{
       esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
       uint8_t *data = (uint8_t *)malloc(RX_BUF_SIZE);
       while (1)
       {
              int len = uart_read_bytes(UART_NUM_2, data, RX_BUF_SIZE, 20 / portTICK_RATE_MS);
              if (len > 0)
              {
                     data[len] = 0;
                     echo_send(UART_NUM_1, "\r\n", 2);
                     char prefix[] = "Sensor Received: [";
                     echo_send(UART_NUM_1, prefix, (sizeof(prefix) - 1));
                     ESP_LOGI(RX_TASK_TAG, "Read UART2 %u bytes: '%s'", len, data);
                     printf("[ ");
                     for (int i = 0; i < len; i++)
                     {
                            printf("0x%.2X ", (uint8_t)data[i]);
                            echo_send(UART_NUM_1, (const char *)&data[i], 1);
                            // Add a Newline character if you get a return charater from paste (Paste tests multibyte receipt/buffer)
                            if (data[i] == '\r')
                            {
                                   echo_send(UART_NUM_1, "\n", 1);
                            }
                     }
                     printf("] \n");
                     echo_send(UART_NUM_1, "]\r\n", 3);

                     if (isConnectedWifi == true)
                     {
                            gpio_set_level(YELLOW_LED, 1);
                            //http_perform_as_stream_reader();
                            const unsigned char *telegraph = data;
                            send_telegraph_http(telegraph);
                            ESP_LOGI(RX_TASK_TAG, "Send Data: %s\n", telegraph);
                            gpio_set_level(YELLOW_LED, 0);
                     }
              }
              else
              {
                     // Echo a "." to show we are alive while we wait for input
                     echo_send(UART_NUM_1, ".", 1);
                     ESP_ERROR_CHECK(uart_wait_tx_done(UART_NUM_1, 10));
              }
       }
       free(data);
}

/**
 * @brief Main function
 * 
 */
void app_main(void)
{
       printf("Start software...\n");
       /* Print chip information */
       esp_chip_info_t chip_info;
       esp_chip_info(&chip_info);
       printf("This is %s chip with %d CPU core(s), WiFi%s%s, ",
              CONFIG_IDF_TARGET,
              chip_info.cores,
              (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
              (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

       printf("silicon revision %d, ", chip_info.revision);

       printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
              (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

       printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());

       //Initialize NVS
       esp_err_t ret = nvs_flash_init();
       if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
       {
              ESP_ERROR_CHECK(nvs_flash_erase());
              ret = nvs_flash_init();
       }
       ESP_ERROR_CHECK(ret);

       ESP_LOGI(INFO, "ESP_WIFI_MODE_STA");
       led_init();
       wifi_init_sta();
       initUart2();
       initUart1();
       rx_task();
       fflush(stdout);
       printf("End software!\n");
}
