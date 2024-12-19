#include <stdio.h>
#include <time.h>
#include <sys/time.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"

#include "nrf24l01p.h"
#include "radio_control.h"

#define NRF24_CE_PIN 25
#define NRF24_IRQ_PIN 26
#define NRF24_CSN_PIN 15

#define NRF24_PAYLOAD_LENGTH 10

static const char *TAG = "radio_control";

void nrf24l01p_set_ce(uint8_t state);
void nrf24l01p_set_cs(uint8_t state);
int8_t nrf24l01p_spi_tx(const uint8_t *tx_data, uint8_t length);
int8_t nrf24l01p_spi_rx(uint8_t *rx_data, uint8_t length);
int8_t nrf24l01p_spi_tx_rx(const uint8_t *tx_data, uint8_t *rx_data, uint8_t length);
void nrf24l01p_irq_handler(void *arg);
int8_t decode_payload(uint8_t *payload, uint8_t length);

Nrf24l01pDevice nrf24_device = {
    .config = {
        .address_width = 5,
        .channel_MHz = 2500,
        .crc_length = NRF24L01P_CRC_1BYTE,
        .data_rate = NRF24L01P_1MBPS,
        .enable_irq_rx_dr = true,
        .enable_irq_max_rt = true,
        .enable_irq_tx_ds = true,
    },
    .rx_config = {
        .address_p0 = NRF24L01P_REG_RX_ADDR_P0_RSTVAL,
        .address_p1 = NRF24L01P_REG_RX_ADDR_P1_RSTVAL,
        .address_p2 = NRF24L01P_REG_RX_ADDR_P2_RSTVAL,
        .address_p3 = NRF24L01P_REG_RX_ADDR_P3_RSTVAL,
        .address_p4 = NRF24L01P_REG_RX_ADDR_P4_RSTVAL,
        .address_p5 = NRF24L01P_REG_RX_ADDR_P5_RSTVAL,
        .data_length = {
            NRF24_PAYLOAD_LENGTH,
            NRF24_PAYLOAD_LENGTH,
            NRF24_PAYLOAD_LENGTH,
            NRF24_PAYLOAD_LENGTH,
            NRF24_PAYLOAD_LENGTH,
            NRF24_PAYLOAD_LENGTH,
        },
        .auto_ack_pipes = 0b00111111,
        .enable_pipes = 0b00111111,
    },
    .tx_config = {
        .address = NRF24L01P_REG_TX_ADDR_RSTVAL,
        .auto_retransmit_count = 3,
        .auto_retransmit_delay_250us = 1,
        .output_power = NRF24L01P_0DBM,
    },
    .interface = {
        .set_cs = &nrf24l01p_set_cs,
        .spi_tx = &nrf24l01p_spi_tx,
        .spi_rx = &nrf24l01p_spi_rx,
        .spi_tx_rx = &nrf24l01p_spi_tx_rx,
    },
};
static Nrf24l01pStatus nrf24_status = NRF24L01P_SUCCESS;
static Nrf24l01pIrq nrf24_irq_sources = {
    .max_rt = false,
    .rx_dr = false,
    .tx_ds = false,
};
static volatile bool nrf24_irq_flag = false;
static bool rx_fifo_empty;
static uint8_t rx_payload[NRF24_PAYLOAD_LENGTH];

static NodeDataSet node_data_set = {
    .node_data = {{
                      .node_id = 0,
                      .app_error = 0,
                      .sht4x_status = 0,
                      .nrf24_status = 0,
                      .temperature_celsius = 0.0,
                      .humidity_pct = 0.0,
                      .vdda_v = 0.0,
                  },
                  {
                      .node_id = 1,
                      .app_error = 0,
                      .sht4x_status = 0,
                      .nrf24_status = 0,
                      .temperature_celsius = 0.0,
                      .humidity_pct = 0.0,
                      .vdda_v = 0.0,
                  },
                  {
                      .node_id = 2,
                      .app_error = 0,
                      .sht4x_status = 0,
                      .nrf24_status = 0,
                      .temperature_celsius = 0.0,
                      .humidity_pct = 0.0,
                      .vdda_v = 0.0,
                  },
                  {
                      .node_id = 3,
                      .app_error = 0,
                      .sht4x_status = 0,
                      .nrf24_status = 0,
                      .temperature_celsius = 0.0,
                      .humidity_pct = 0.0,
                      .vdda_v = 0.0,
                  },
                  {
                      .node_id = 4,
                      .app_error = 0,
                      .sht4x_status = 0,
                      .nrf24_status = 0,
                      .temperature_celsius = 0.0,
                      .humidity_pct = 0.0,
                      .vdda_v = 0.0,
                  },
                  {
                      .node_id = 5,
                      .app_error = 0,
                      .sht4x_status = 0,
                      .nrf24_status = 0,
                      .temperature_celsius = 0.0,
                      .humidity_pct = 0.0,
                      .vdda_v = 0.0,
                  }},
};

spi_bus_config_t spi_bus_config = {
    .mosi_io_num = 13,
    .miso_io_num = 12,
    .sclk_io_num = 14,
    .max_transfer_sz = 32,
    .flags = 0,
    .isr_cpu_id = APP_CPU_NUM,
    .intr_flags = 0,
};
spi_device_interface_config_t nrf24_spi_device_config = {
    .command_bits = 0,
    .address_bits = 0,
    .dummy_bits = 0,
    .mode = 0,
    .clock_source = SPI_CLK_SRC_DEFAULT,
    .duty_cycle_pos = 128,
    .cs_ena_pretrans = 0,
    .cs_ena_posttrans = 0,
    .clock_speed_hz = 100000,
    .input_delay_ns = 0,
    .spics_io_num = NRF24_CSN_PIN,
    .flags = 0,
    .queue_size = 1,
    .pre_cb = NULL,
    .post_cb = NULL,
};
spi_device_handle_t nrf24_spi_handle;

void task_nrf24_control(void *pvParameters)
{
    node_data_set.mutex = xSemaphoreCreateMutex();

    ESP_ERROR_CHECK(gpio_set_direction(NRF24_CE_PIN, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(NRF24_CE_PIN, GPIO_FLOATING));
    ESP_ERROR_CHECK(gpio_set_level(NRF24_CE_PIN, 0));

    ESP_ERROR_CHECK(gpio_set_direction(NRF24_CSN_PIN, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(NRF24_CSN_PIN, GPIO_FLOATING));
    ESP_ERROR_CHECK(gpio_set_level(NRF24_CSN_PIN, 1));

    ESP_ERROR_CHECK(gpio_set_direction(NRF24_IRQ_PIN, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(NRF24_IRQ_PIN, GPIO_FLOATING));
    ESP_ERROR_CHECK(gpio_intr_enable(NRF24_IRQ_PIN));
    ESP_ERROR_CHECK(gpio_set_intr_type(NRF24_IRQ_PIN, GPIO_INTR_NEGEDGE));
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1));
    ESP_ERROR_CHECK(gpio_isr_handler_add(NRF24_IRQ_PIN, &nrf24l01p_irq_handler, (void *)NRF24_IRQ_PIN));

    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &spi_bus_config, SPI_DMA_DISABLED));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &nrf24_spi_device_config, &nrf24_spi_handle));

    nrf24_status = nrf24l01p_init_prx(&nrf24_device);
    ESP_LOGI(TAG, "[nRF24] init_prx: status = %d", nrf24_status);

    nrf24_status = nrf24l01p_clear_status_flags(&nrf24_device,
                                                (uint8_t)NRF24L01P_REG_STATUS_RX_DR |
                                                    NRF24L01P_REG_STATUS_TX_DS |
                                                    NRF24L01P_REG_STATUS_MAX_RT);
    ESP_LOGI(TAG, "[nRF24] clear status flags: status = %d", nrf24_status);

    nrf24_status = nrf24l01p_power_up(&nrf24_device);
    ESP_LOGI(TAG, "[nRF24] power_up: status = %d", nrf24_status);

    vTaskDelay(10 / portTICK_PERIOD_MS);

    nrf24l01p_set_ce(1);

    while (1)
    {
        vTaskDelay(10 / portTICK_PERIOD_MS);

        if (nrf24_irq_flag)
        {
            nrf24_irq_flag = false;

            nrf24_status = nrf24l01p_get_and_clear_irq_flags(&nrf24_device, &nrf24_irq_sources);
            ESP_LOGI(TAG, "[nRF24] irq flags: MAX_RT = %d, RX_DR = %d, TX_DS = %d, status = %d",
                     nrf24_irq_sources.max_rt, nrf24_irq_sources.rx_dr, nrf24_irq_sources.tx_ds, nrf24_status);

            if (nrf24_irq_sources.rx_dr)
            {
                while (true)
                {
                    nrf24_status = nrf24l01p_is_rx_fifo_empty(&nrf24_device, &rx_fifo_empty);
                    ESP_LOGI(TAG, "[nRF24] is RX FIFO empty: %d, status = %d", rx_fifo_empty, nrf24_status);

                    if (rx_fifo_empty)
                    {
                        break;
                    }

                    nrf24_status = nrf24l01p_rx_receive(&nrf24_device, rx_payload);
                    ESP_LOGI(TAG, "[nRF24] read RX FIFO: status = %d", nrf24_status);
                    decode_payload(rx_payload, NRF24_PAYLOAD_LENGTH);
                    ESP_LOGI(TAG, "[nRF24] temperature: %.2f °C, humidity: %.2f %%, voltage = %.3f V",
                             node_data_set.node_data[0].temperature_celsius,
                             node_data_set.node_data[0].humidity_pct,
                             node_data_set.node_data[0].vdda_v);
                }
            }
        }
    }
}

void nrf24l01p_set_ce(uint8_t state)
{
    gpio_set_level(NRF24_CE_PIN, state);
}

void nrf24l01p_set_cs(uint8_t state)
{
    // Leave empty, as CS is managed by ESP-IDF SPI driver
}

int8_t nrf24l01p_spi_tx(const uint8_t *tx_data, uint8_t length)
{
    spi_transaction_t transaction = {
        .flags = 0,
        .cmd = 0,
        .addr = 0,
        .length = length * 8,
        .rxlength = 0,
        .tx_buffer = tx_data,
        .rx_buffer = NULL,
    };
    ESP_ERROR_CHECK(spi_device_transmit(nrf24_spi_handle, &transaction));
    return 0;
}

int8_t nrf24l01p_spi_rx(uint8_t *rx_data, uint8_t length)
{
    spi_transaction_t transaction = {
        .flags = 0,
        .cmd = 0,
        .addr = 0,
        .length = length * 8,
        .rxlength = length * 8,
        .tx_buffer = NULL,
        .rx_buffer = rx_data,
    };
    ESP_ERROR_CHECK(spi_device_transmit(nrf24_spi_handle, &transaction));
    return 0;
}

int8_t nrf24l01p_spi_tx_rx(const uint8_t *tx_data, uint8_t *rx_data, uint8_t length)
{
    spi_transaction_t transaction = {
        .flags = 0,
        .cmd = 0,
        .addr = 0,
        .length = length * 8,
        .rxlength = length * 8,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };
    ESP_ERROR_CHECK(spi_device_transmit(nrf24_spi_handle, &transaction));
    return 0;
}

void nrf24l01p_irq_handler(void *arg)
{
    (void)arg;
    if (gpio_get_level(NRF24_IRQ_PIN) == 0)
        nrf24_irq_flag = true;
}

int8_t decode_payload(uint8_t *payload, uint8_t length)
{
    uint8_t node_id = payload[0];
    if (node_id > 5)
    {
        return -1;
    }

    if (xSemaphoreTake(node_data_set.mutex, portMAX_DELAY) == pdTRUE) // Attempt to acquire the mutex
    {
        NodeData *data = &node_data_set.node_data[node_id];
        data->app_error = (int8_t)payload[1];
        data->sht4x_status = payload[2];
        data->nrf24_status = payload[3];

        int16_t t_16 = ((int16_t)(payload[4] << 8) + (int16_t)payload[5]);
        int16_t rh_16 = ((int16_t)(payload[6] << 8) + (int16_t)payload[7]);
        uint16_t v_16 = (((uint16_t)payload[8] << 8) + (uint16_t)payload[9]);

        data->temperature_celsius = (float)t_16 / 100.0;
        data->humidity_pct = (float)rh_16 / 100.0;
        data->vdda_v = (float)v_16 / 1000.0;

        gettimeofday(&data->timestamp, NULL);

        // Release mutex
        xSemaphoreGive(node_data_set.mutex);
        return 0;
    }

    return -1;
}

int8_t get_node_data(NodeDataSet *target)
{
    if (target == NULL)
    {
        return -1; // Invalid pointer
    }

    if (xSemaphoreTake(node_data_set.mutex, portMAX_DELAY) == pdTRUE) // Attempt to acquire the mutex
    {
        *target = node_data_set;             // Copy data to the caller's buffer
        xSemaphoreGive(node_data_set.mutex); // Release the mutex
        return 0;
    }

    return -1; // If we couldn't acquire the mutex (shouldn't happen with portMAX_DELAY)
}