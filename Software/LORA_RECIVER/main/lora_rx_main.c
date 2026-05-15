#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"


/* UART pins */
#define TXD_PIN             GPIO_NUM_14
#define RXD_PIN             GPIO_NUM_15
#define UART_PORT           UART_NUM_1

/* Buffers */
#define RX_BUF_SIZE         1024
#define CMD_BUF_SIZE        128
#define PACKET_BUF_SIZE     512
#define DATA_BUF_SIZE       64

/* Timeouts */
#define TIMEOUT_SHORT_MS    2000    /* Short AT command timeout       */
#define RX_WINDOW_MS        20000   /* RX listen window per cycle     */
#define PAUSE_BETWEEN_RX_MS 500     /* Pause between RX cycles        */


#define LORA_FREQ           868000000
#define LORA_POWER          14
#define LORA_BW             4
#define LORA_SF             12
#define LORA_CR             1
#define LORA_PAYLOAD_LEN    16

/* Logging tag */
static const char *TAG = "RX";

/* Runtime statistics */
static int packets_received = 0;
static int timeout_count    = 0;
static int rx_errors        = 0;


static void uart_init(void)
{
    const uart_config_t config = {
        .baud_rate  = 115200,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, RX_BUF_SIZE * 2,
                                        0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, TXD_PIN, RXD_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "UART init: baud=115200, TX=GPIO%d, RX=GPIO%d",
             TXD_PIN, RXD_PIN);
}


static bool send_at_command(const char *cmd, uint32_t timeout_ms)
{
    uart_flush_input(UART_PORT);
    uart_write_bytes(UART_PORT, cmd, strlen(cmd));
    uart_write_bytes(UART_PORT, "\r\n", 2);
    ESP_LOGI(TAG, ">>> %s", cmd);

    uint8_t rx_buf[RX_BUF_SIZE];
    int     total   = 0;
    int64_t t_start = esp_timer_get_time() / 1000;

    memset(rx_buf, 0, sizeof(rx_buf));

    while ((esp_timer_get_time() / 1000 - t_start) < timeout_ms) {

        int len = uart_read_bytes(UART_PORT, rx_buf + total,
                                  1, pdMS_TO_TICKS(10));

        if (len > 0 && total < (RX_BUF_SIZE - 1)) {
            printf("%c", rx_buf[total]);
            total++;
            rx_buf[total] = '\0';

            if (strstr((char *)rx_buf, "\r\nOK\r\n") ||
                strstr((char *)rx_buf, "\nOK\n")     ||
                strstr((char *)rx_buf, "OK\r\n"))
            {
                ESP_LOGI(TAG, "<<< OK");
                return true;
            }

            if (strstr((char *)rx_buf, "PARAM_ERROR") ||
                strstr((char *)rx_buf, "BUSY_ERROR")  ||
                strstr((char *)rx_buf, "ERROR"))
            {
                ESP_LOGW(TAG, "<<< ERROR in response");
                return false;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return false; /* timeout */
}


static bool lora_p2p_init(void)
{
    ESP_LOGI(TAG, "=== Initializing LoRa P2P (RX) ===");

    /* Check module communication */
    bool module_active = false;
    for (int i = 0; i < 3; i++) {
        if (send_at_command("ATQ", TIMEOUT_SHORT_MS)) {
            module_active = true;
            break;
        }
        ESP_LOGW(TAG, "Module not responding, attempt %d/3...", i + 1);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    if (!module_active) {
        ESP_LOGE(TAG, "ERROR: KG200Z not responding!");
        return false;
    }

    /* Enable P2P mode */
    if (!send_at_command("AT+QP2P=1", TIMEOUT_SHORT_MS)) {
        ESP_LOGE(TAG, "ERROR: AT+QP2P=1 rejected!");
        return false;
    }

    /* RF configuration – MUST match the transmitter exactly! */
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

    ESP_LOGI(TAG, "P2P init OK | Listening on %dMHz | SF%d | BW125",
             LORA_FREQ / 1000000, LORA_SF);
    return true;
}


static void parse_rx_data(const char *rx_buf)
{
    /* Extract payload data (after "buf: ") */
    const char *buf_ptr = strstr(rx_buf, "buf: ");
    if (buf_ptr) {
        buf_ptr += 5; /* skip "buf: " */
        char received[DATA_BUF_SIZE] = {0};
        int i = 0;

        while (*buf_ptr && *buf_ptr != '\r' && *buf_ptr != '\n'
               && i < (int)(sizeof(received) - 1)) {
            received[i++] = *buf_ptr++;
        }
        ESP_LOGI(TAG, "  Data      : %s", received);
    } else {
        ESP_LOGW(TAG, "  Data      : (could not parse)");
    }

    /* Extract RSSI value */
    const char *rssi_ptr = strstr(rx_buf, "RssiValue=");
    if (rssi_ptr) {
        rssi_ptr += 10; /* skip "RssiValue=" */
        int rssi = atoi(rssi_ptr);
        ESP_LOGI(TAG, "  RSSI      : %d dBm", rssi);
    }

    /* Extract SNR value */
    const char *snr_ptr = strstr(rx_buf, "SnrValue=");
    if (snr_ptr) {
        snr_ptr += 9;
        int snr = atoi(snr_ptr);
        ESP_LOGI(TAG, "  SNR       : %d dB", snr);
    }
}


static bool wait_for_packet(void)
{
    /* Flush the RX buffer before listening */
    uart_flush_input(UART_PORT);

    /* Send AT+QTRX=1 to open a receive window for 1 packet */
    uart_write_bytes(UART_PORT, "AT+QTRX=1\r\n", 11);
    ESP_LOGI(TAG, ">>> AT+QTRX=1 (waiting %ds...)", RX_WINDOW_MS / 1000);

    uint8_t packet_buf[PACKET_BUF_SIZE];
    int     total   = 0;
    bool    received = false;
    int64_t t_start = esp_timer_get_time() / 1000;

    memset(packet_buf, 0, sizeof(packet_buf));

    while ((esp_timer_get_time() / 1000 - t_start) < RX_WINDOW_MS) {

        int len = uart_read_bytes(UART_PORT,
                                  packet_buf + total,
                                  1,
                                  pdMS_TO_TICKS(10));

        if (len > 0 && total < (PACKET_BUF_SIZE - 1)) {
            printf("%c", packet_buf[total]);
            total++;
            packet_buf[total] = '\0';

            /*
             * "buf: " only appears in the response when a packet has
             * been physically received over the LoRa radio.
             */
            if (strstr((char *)packet_buf, "buf: ")) {
                received = true;
            }

            /*
             * "OK" marks the end of the module response.
             * Exit the loop regardless of whether a packet was received.
             */
            if (strstr((char *)packet_buf, "\r\nOK\r\n") ||
                strstr((char *)packet_buf, "OK\r\n"))
            {
                break;
            }

            /* ERROR response – something went wrong */
            if (strstr((char *)packet_buf, "ERROR")) {
                ESP_LOGW(TAG, "Error during RX command");
                rx_errors++;
                return false;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    /* Evaluate result */
    if (received) {
        packets_received++;
        ESP_LOGI(TAG, "PACKET RECEIVED #%d", packets_received);
        parse_rx_data((char *)packet_buf);
        return true;
    } else {
        /* Timeout – normal when waiting for infrequent TX packets */
        timeout_count++;
        ESP_LOGI(TAG, "[IDLE] No packet (timeout #%d, total received: %d)",
                 timeout_count, packets_received);
        return false;
    }
}


void app_main(void)
{
    printf("\n");
    ESP_LOGI(TAG, "      LoRa RECEIVER  |  ACTIVE      ");

    uart_init();

    /* Wait for KG200Z module to boot */
    ESP_LOGI(TAG, "Waiting for KG200Z module boot (3s)...");
    vTaskDelay(pdMS_TO_TICKS(3000));

    /* Initialize LoRa module – retry indefinitely on failure */
    while (!lora_p2p_init()) {
        ESP_LOGE(TAG, "Initialization failed, retrying in 5s...");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    ESP_LOGI(TAG, "Starting to listen...");
    ESP_LOGI(TAG, "Waiting for packet on 868MHz | SF12 | BW125kHz");

    /* Main receive loop */
    while (1) {
        wait_for_packet();

        /*
         * Short pause between RX windows:
         *  - prevents FreeRTOS task starvation
         */
        vTaskDelay(pdMS_TO_TICKS(PAUSE_BETWEEN_RX_MS));
    }
}
