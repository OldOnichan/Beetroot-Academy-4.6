#include <cstdint>
#include <cstdio>
#include <cstring>

#include "driver/uart.h"
#include "driver/uhci.h"
#include "esp_adc/adc_continuous.h"
#include "esp_attr.h"
#include "esp_err.h"

static constexpr adc_channel_t adc_channel = ADC_CHANNEL_8;
static constexpr unsigned sample_rate = 20000;
static constexpr unsigned report_samples = 4000;
static constexpr unsigned samples_between_readings = report_samples / 4;
static constexpr unsigned frame_size = 512;

alignas(4) static DMA_ATTR uint8_t message[128];

static uhci_controller_handle_t init_uart()
{
    uart_config_t serial{};
    serial.baud_rate = 115200;
    serial.data_bits = UART_DATA_8_BITS;
    serial.parity = UART_PARITY_DISABLE;
    serial.stop_bits = UART_STOP_BITS_1;
    serial.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    serial.source_clk = UART_SCLK_DEFAULT;
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &serial));

    uhci_controller_config_t config{};
    config.uart_port = UART_NUM_0;
    config.tx_trans_queue_depth = 2;
    config.max_transmit_size = sizeof(message);
    config.max_receive_internal_mem = 1024;

    uhci_controller_handle_t uart;
    ESP_ERROR_CHECK(uhci_new_controller(&config, &uart));
    return uart;
}

static adc_continuous_handle_t init_adc()
{
    adc_continuous_handle_cfg_t buffer{};
    buffer.max_store_buf_size = 4096;
    buffer.conv_frame_size = frame_size;
    buffer.flags.flush_pool = 1;

    adc_continuous_handle_t adc;
    ESP_ERROR_CHECK(adc_continuous_new_handle(&buffer, &adc));

    adc_digi_pattern_config_t input{};
    input.atten = ADC_ATTEN_DB_12;
    input.channel = adc_channel;
    input.unit = ADC_UNIT_1;
    input.bit_width = ADC_BITWIDTH_12;

    adc_continuous_config_t config{};
    config.sample_freq_hz = sample_rate;
    config.conv_mode = ADC_CONV_SINGLE_UNIT_1;
    config.format = ADC_DIGI_OUTPUT_FORMAT_TYPE2;
    config.pattern_num = 1;
    config.adc_pattern = &input;
    ESP_ERROR_CHECK(adc_continuous_config(adc, &config));
    ESP_ERROR_CHECK(adc_continuous_start(adc));
    return adc;
}

struct Report {
    unsigned sum = 0;
    unsigned count = 0;
    unsigned minimum = 4095;
    unsigned maximum = 0;
    unsigned readings[4]{};
};

static void add_sample(Report &report, unsigned value)
{
    if (report.count % samples_between_readings == 0) {
        report.readings[report.count / samples_between_readings] = value;
    }
    if (value < report.minimum) {
        report.minimum = value;
    }
    if (value > report.maximum) {
        report.maximum = value;
    }
    report.sum += value;
    ++report.count;
}

static void send_report(uhci_controller_handle_t uart, const Report &report)
{
    const int length = std::snprintf(
        reinterpret_cast<char *>(message), sizeof(message),
        "ADC %u %u %u %u | min=%u max=%u pp=%u avg=%u n=%u\r\n",
        report.readings[0], report.readings[1], report.readings[2], report.readings[3],
        report.minimum, report.maximum, report.maximum - report.minimum,
        report.sum / report.count, report.count);

    if (length > 0 && static_cast<unsigned>(length) < sizeof(message)) {
        ESP_ERROR_CHECK(uhci_transmit(uart, message, length));
        ESP_ERROR_CHECK(uhci_wait_all_tx_transaction_done(uart, 1000));
    }
}

extern "C" void app_main()
{
    const auto uart = init_uart();
    const auto adc = init_adc();
    alignas(4) uint8_t block[frame_size];
    Report report;

    while (true) {
        uint32_t bytes = 0;
        const esp_err_t result = adc_continuous_read(adc, block, sizeof(block), &bytes, 1000);
        if (result == ESP_ERR_TIMEOUT) {
            continue;
        }
        ESP_ERROR_CHECK(result);

        for (unsigned i = 0; i + SOC_ADC_DIGI_RESULT_BYTES <= bytes; i += SOC_ADC_DIGI_RESULT_BYTES) {
            adc_digi_output_data_t sample{};
            std::memcpy(&sample, block + i, SOC_ADC_DIGI_RESULT_BYTES);
            if (sample.type2.channel != adc_channel || sample.type2.unit != ADC_UNIT_1) {
                continue;
            }

            add_sample(report, sample.type2.data);
            if (report.count == report_samples) {
                send_report(uart, report);
                report = Report{};
            }
        }
    }
}
