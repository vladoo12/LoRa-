#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"


/* UART pins */
#define TXD_PIN             GPIO_NUM_14
#define RXD_PIN             GPIO_NUM_15
#define UART_PORT           UART_NUM_1

/* Buffers */
#define RX_BUF_SIZE         1024    /* bytes for UART RX buffer          */
#define CMD_BUF_SIZE        128     /* bytes for AT command string        */
#define DATA_BUF_SIZE       64      /* bytes for sensor payload           */

/*
 * Timeouts (ms):
 *   SHORT – for commands that respond immediately (ATQ, AT+QP2P, AT+QTCONF)
 *   LONG  – for AT+QTTX which waits for the physical TX transmission to finish
 */
#define TIMEOUT_SHORT_MS    2000
#define TIMEOUT_LONG_MS     8000

/* Wait after transmission (ms) for the module to finish TX before sleeping */
#define WAIT_AFTER_TX_MS    3000

/*
 * Deep sleep duration: 24 hours in microseconds
 */
#define SLEEP_DURATION_US   (24ULL * 60ULL * 60ULL * 1000000ULL)



#define LORA_FREQ           868000000   /* Hz – EU868 band             */
#define LORA_POWER          14          /* dBm – max for EU868         */
#define LORA_BW             4           /* 4 = 125 kHz (LoRa)         */
#define LORA_SF             12          /* SF12 = max range, min speed */
#define LORA_CR             1           /* 1 = 4/5 code rate           */
#define LORA_PAYLOAD_LEN    16          /* bytes – fixed size          */


RTC_DATA_ATTR static int wake_count        = 0;  /* number of wakeups          */
RTC_DATA_ATTR static int total_sent        = 0;  /* successfully sent packets  */
RTC_DATA_ATTR static int total_errors      = 0;  /* failed attempts            */

static const char *TAG = "TX";


static void uart_init(void)
{
    const uart_config_t config = {
        .baud_rate  = 115200,                    /* KG200Z default!    */
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(
        UART_PORT,
        RX_BUF_SIZE * 2,   /* RX buffer size */
        0,                  /* TX buffer = 0 → blocking mode */
        0,                  /* No event queue */
        NULL,
        0
    ));

    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &config));

    ESP_ERROR_CHECK(uart_set_pin(
        UART_PORT,
        TXD_PIN,             /* ESP32 TX → KG200Z RX */
        RXD_PIN,             /* ESP32 RX ← KG200Z TX */
        UART_PIN_NO_CHANGE,  /* RTS – not used */
        UART_PIN_NO_CHANGE   /* CTS – not used */
    ));

    ESP_LOGI(TAG, "UART init: baud=115200, TX=GPIO%d, RX=GPIO%d",
             TXD_PIN, RXD_PIN);
}


static bool send_at_command(const char *cmd, uint32_t timeout_ms)
{
    /* STEP 1: Flush any stale data in the RX buffer */
    uart_flush_input(UART_PORT);

    /* STEP 2: Send the command with CR+LF terminator */
    uart_write_bytes(UART_PORT, cmd, strlen(cmd));
    uart_write_bytes(UART_PORT, "\r\n", 2);

    /* STEP 3: Read the response byte by byte */
    uint8_t rx_buf[RX_BUF_SIZE];
    int     total_bytes = 0;
    int64_t t_start     = esp_timer_get_time() / 1000; /* ms */

    memset(rx_buf, 0, sizeof(rx_buf));

    while ((esp_timer_get_time() / 1000 - t_start) < timeout_ms) {

        int received = uart_read_bytes(
            UART_PORT,
            rx_buf + total_bytes,
            1,                       /* read one byte at a time */
            pdMS_TO_TICKS(10)
        );

        if (received > 0 && total_bytes < (RX_BUF_SIZE - 1)) {
            printf("%c", rx_buf[total_bytes]);
            total_bytes++;
            rx_buf[total_bytes] = '\0'; /* null-terminate for strstr */

            /* Check for OK response */
            if (strstr((char *)rx_buf, "\r\nOK\r\n") ||
                strstr((char *)rx_buf, "\nOK\n")     ||
                strstr((char *)rx_buf, "OK\r\n"))
            {
                return true;
            }

            /* Check for error response */
            if (strstr((char *)rx_buf, "PARAM_ERROR") ||
                strstr((char *)rx_buf, "BUSY_ERROR")  ||
                strstr((char *)rx_buf, "ERROR"))
            {
                return false;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return false; /* timeout */
}


static bool lora_p2p_init(void)
{
    /* --- Step 1: Check if module is responsive --- */
    bool module_active = false;
    for (int i = 0; i < 3; i++) {
        if (send_at_command("ATQ", TIMEOUT_SHORT_MS)) {
            module_active = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    if (!module_active) {
        ESP_LOGE(TAG, "ERROR: KG200Z not responding to ATQ!");
        return false;
    }

    /* --- Step 2: Enable P2P mode --- */
    if (!send_at_command("AT+QP2P=1", TIMEOUT_SHORT_MS)) {
        ESP_LOGE(TAG, "ERROR: AT+QP2P=1 rejected!");
        return false;
    }

    /* --- Step 3: Set RF parameters --- */
    char conf_cmd[CMD_BUF_SIZE];
    snprintf(conf_cmd, sizeof(conf_cmd),
        "AT+QTCONF=%d:%d:%d:%d:%d:0:0:1:%d:25000:2:3",
        LORA_FREQ,
        LORA_POWER,
        LORA_BW,
        LORA_SF,
        LORA_CR,
        LORA_PAYLOAD_LEN
    );

    if (!send_at_command(conf_cmd, TIMEOUT_SHORT_MS)) {
        ESP_LOGE(TAG, "ERROR: AT+QTCONF rejected!");
        return false;
    }

    ESP_LOGI(TAG, "P2P init OK | freq=%dMHz | SF%d | BW125 | 14dBm",
             LORA_FREQ / 1000000, LORA_SF);
    return true;
}


static bool send_sensor_data(void)
{
    /* Generate simulated sensor readings */
    int temp     = (rand() % 81) - 30;  /* -30°C to +50°C */
    int humidity = rand() % 101;         /* 0% to 100%     */

    char payload[DATA_BUF_SIZE];
    char cmd[CMD_BUF_SIZE];

    /* Format the payload string */
    snprintf(payload, sizeof(payload), "T:%dC,H:%d%%", temp, humidity);
    ESP_LOGI(TAG, "Sending: %s", payload);

    /* Load data into module TX buffer */
    snprintf(cmd, sizeof(cmd), "AT+QTDA=%s", payload);
    if (!send_at_command(cmd, TIMEOUT_SHORT_MS)) {
        ESP_LOGE(TAG, "AT+QTDA not accepted!");
        return false;
    }

    /* Trigger RF transmission */
    if (!send_at_command("AT+QTTX=1", TIMEOUT_LONG_MS)) {
        ESP_LOGE(TAG, "AT+QTTX not accepted!");
        return false;
    }

    ESP_LOGI(TAG, "Packet sent: %s", payload);
    return true;
}


static void go_to_sleep(void)
{
    ESP_LOGI(TAG, "=== Preparing for sleep ===");

    /* Disable P2P mode on the module */
    send_at_command("AT+QP2P=0", TIMEOUT_SHORT_MS);

    /* Short delay to let the command complete */
    vTaskDelay(pdMS_TO_TICKS(500));

    /* Log session statistics */
    ESP_LOGI(TAG, "Statistics:");
    ESP_LOGI(TAG, "  Total wakeups  : %d", wake_count);
    ESP_LOGI(TAG, "  Packets sent   : %d", total_sent);
    ESP_LOGI(TAG, "  Errors         : %d", total_errors);
    ESP_LOGI(TAG, "  Sleeping 24h...");

    /* Flush UART before sleeping */
    uart_flush(UART_PORT);
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Configure wakeup timer */
    esp_sleep_enable_timer_wakeup(SLEEP_DURATION_US);

    /* Enter deep sleep – does not return */
    esp_deep_sleep_start();
}


void app_main(void)
{
    /* Increment wakeup counter (persists across deep sleep) */
    wake_count++;

    printf("\n");
    ESP_LOGI(TAG, "LoRa TRANSMITTER  |  Wakeup #%d", wake_count);

    /* Log wakeup cause */
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
        ESP_LOGI(TAG, "Wakeup cause: 24h timer (normal operation)");
    } else {
        ESP_LOGI(TAG, "Wakeup cause: power-on or reset");
    }

    uart_init();

    /* Wait for KG200Z module to boot */
    ESP_LOGI(TAG, "Waiting for KG200Z module boot (3s)...");
    vTaskDelay(pdMS_TO_TICKS(3000));

    /* Initialize LoRa module */
    if (!lora_p2p_init()) {
        ESP_LOGE(TAG, "Initialization failed – going to sleep");
        total_errors++;
        go_to_sleep();
        return;
    }

    /* Short delay before transmission */
    vTaskDelay(pdMS_TO_TICKS(500));

    /* Send sensor data */
    if (send_sensor_data()) {
        total_sent++;
        ESP_LOGI(TAG, "Send OK (total: %d)", total_sent);
    } else {
        total_errors++;
        ESP_LOGW(TAG, "Send failed (errors: %d)", total_errors);
    }

    /* Wait for TX to complete physically */
    ESP_LOGI(TAG, "Waiting for TX to finish (%dms)...", WAIT_AFTER_TX_MS);
    vTaskDelay(pdMS_TO_TICKS(WAIT_AFTER_TX_MS));

    /* Enter sleep */
    go_to_sleep();
}
