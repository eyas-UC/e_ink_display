#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_err.h"

// Pin definitions
#define EPAPER_RST_PIN     GPIO_NUM_17
#define EPAPER_DC_PIN      GPIO_NUM_16
#define EPAPER_CS_PIN      GPIO_NUM_5
#define EPAPER_BUSY_PIN    GPIO_NUM_4
#define EPAPER_MOSI_PIN    GPIO_NUM_23
#define EPAPER_SCLK_PIN    GPIO_NUM_18

// EPD constants for 4.2 inch 400x300
#define EPD_WIDTH  400
#define EPD_HEIGHT 300
#define EPD_BUFFER_SIZE ((EPD_WIDTH * EPD_HEIGHT) / 8)

// Maximum chunk size for SPI transfers (adjust based on your ESP32 model)
#define SPI_MAX_TRANSFER_SIZE 4096

// Commands for 4.2 inch e-paper (GDEW042T2 or similar)
#define DRIVER_OUTPUT_CONTROL               0x01
#define BOOSTER_SOFT_START_CONTROL          0x3C
#define GATE_SCAN_START_POSITION            0x0F
#define DEEP_SLEEP_MODE                     0x10
#define DATA_ENTRY_MODE_SETTING             0x11
#define SW_RESET                            0x12
#define TEMPERATURE_SENSOR_READ             0X18
#define TEMPERATURE_SENSOR_CONTROL          0x1A
#define MASTER_ACTIVATION                   0x20
#define DISPLAY_UPDATE_CONTROL_2            0x22
#define WRITE_RAM                           0x24
#define WRITE_VCOM_REGISTER                 0x2C
#define SET_DUMMY_LINE_PERIOD               0x3A
#define SET_GATE_TIME                       0x3B
#define SET_RAM_X_ADDRESS_START_END_POSITION 0x44
#define SET_RAM_Y_ADDRESS_START_END_POSITION 0x45
#define SET_RAM_X_ADDRESS_COUNTER           0x4E
#define SET_RAM_Y_ADDRESS_COUNTER           0x4F

static const char *TAG = "Epaper";

// SPI handle
spi_device_handle_t spi;

// Font size enum
typedef enum {
    FONT_SIZE_1X = 1,
    FONT_SIZE_2X = 2,
    FONT_SIZE_3X = 3,
    FONT_SIZE_4X = 4
} font_size_t;

// Function prototypes
static void epaper_gpio_init(void);
static esp_err_t epaper_spi_init(void);
static void epaper_write_command(uint8_t command);
static void epaper_write_data(uint8_t data);
static void epaper_write_data_buffer_chunked(const uint8_t command, const uint8_t *data, uint32_t length);
static void epaper_busy_wait(void);
static void epaper_reset(void);
static void epaper_init(void);
static void epaper_set_memory_area(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end);
// static void epaper_set_memory_pointer(uint16_t x, uint16_t y);
static void epaper_display(const uint8_t *image_data);
static void epaper_clear(void);
static void epaper_sleep(void);
static void writeScreenBuffer(uint8_t command, uint8_t value);
static void full_update();
static void epaper_draw_string_to_buffer(uint8_t *image_data, const char *text, uint16_t x, uint16_t y, font_size_t size);
static void epaper_draw_string(const char *text, uint16_t x, uint16_t y, font_size_t size);


// GPIO initialization
static void epaper_gpio_init(void)
{
    // Reset pin
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << EPAPER_RST_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(EPAPER_RST_PIN, 1);
    
    // DC pin
    io_conf.pin_bit_mask = (1ULL << EPAPER_DC_PIN);
    gpio_config(&io_conf);
    gpio_set_level(EPAPER_DC_PIN, 0);
    
    // CS pin - let SPI driver handle it
    io_conf.pin_bit_mask = (1ULL << EPAPER_CS_PIN);
    gpio_config(&io_conf);
    gpio_set_level(EPAPER_CS_PIN, 1);
    
    // BUSY pin (input with pullup)
    io_conf.pin_bit_mask = (1ULL << EPAPER_BUSY_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
    
    ESP_LOGI(TAG, "GPIO initialized");
}

// SPI initialization with larger transfer size
static esp_err_t epaper_spi_init(void)
{
    esp_err_t ret;
    
    // Configure SPI bus with larger buffer
    spi_bus_config_t buscfg = {
        .mosi_io_num = EPAPER_MOSI_PIN,
        .miso_io_num = -1,           // Not used for e-paper
        .sclk_io_num = EPAPER_SCLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = SPI_MAX_TRANSFER_SIZE,
        .flags = 0,
        .intr_flags = 0,
    };
    
    // Configure SPI device
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 2 * 1000 * 1000,  // 2MHz for stability
        .mode = 0,                          // SPI mode 0
        .spics_io_num = EPAPER_CS_PIN,
        .queue_size = 7,
        .flags = SPI_DEVICE_HALFDUPLEX,
        .pre_cb = NULL,
    };
    
    // Initialize SPI bus
    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI2 bus: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Add device to SPI bus
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "SPI initialized successfully with max transfer size: %d bytes", SPI_MAX_TRANSFER_SIZE);
    return ESP_OK;
}

// Write command to EPD
static void epaper_write_command(uint8_t command)
{
    esp_err_t ret;
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &command,
        .flags = 0,
    };
    
    gpio_set_level(EPAPER_CS_PIN, 0);  // CS start comms
    gpio_set_level(EPAPER_DC_PIN, 0);  // DC low for command
    ret = spi_device_polling_transmit(spi, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write command 0x%02X: %s", command, esp_err_to_name(ret));
    }
    gpio_set_level(EPAPER_CS_PIN, 1);  // end comms
    gpio_set_level(EPAPER_DC_PIN, 1);  // end comms
    vTaskDelay(pdMS_TO_TICKS(1));
}

// Write data to EPD
static void epaper_write_data(uint8_t data)
{
    esp_err_t ret;
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &data,
        .flags = 0,
    };
    gpio_set_level(EPAPER_CS_PIN, 0);  // CS start comms
    gpio_set_level(EPAPER_DC_PIN, 1);  // DC high for data
    ret = spi_device_polling_transmit(spi, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write data 0x%02X: %s", data, esp_err_to_name(ret));
    }
    gpio_set_level(EPAPER_DC_PIN, 0);  // DC low for command
    vTaskDelay(pdMS_TO_TICKS(2));
}

// Write data buffer to EPD in chunks
static void epaper_write_data_buffer_chunked(const uint8_t command, const uint8_t *data, uint32_t length)
{
    if (length == 0) return;
    
    epaper_set_memory_area(0, 0, EPD_WIDTH - 1, EPD_HEIGHT - 1);
    uint32_t bytes_sent = 0;
    epaper_write_command(command);
    
    gpio_set_level(EPAPER_CS_PIN, 0);  // start comms
    gpio_set_level(EPAPER_DC_PIN, 1);  // DC HIGH for data bytes
    
    while (bytes_sent < length) {
        uint32_t chunk_size = length - bytes_sent;
        
        // Limit chunk size to maximum transfer size
        if (chunk_size > SPI_MAX_TRANSFER_SIZE) {
            chunk_size = SPI_MAX_TRANSFER_SIZE;
        }
        
        esp_err_t ret;
        spi_transaction_t t = {
            .length = chunk_size * 8,  // Convert bytes to bits
            .tx_buffer = data + bytes_sent,
            .flags = 0,
        };
        ret = spi_device_polling_transmit(spi, &t);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write data chunk (%d bytes): %s", chunk_size, esp_err_to_name(ret));
            break;
        }
        
        
        bytes_sent += chunk_size;
        ESP_LOGD(TAG, "Sent %d/%d bytes", bytes_sent, length);
    }
    gpio_set_level(EPAPER_CS_PIN, 1);  // end comms
    
    ESP_LOGI(TAG, "Total data sent: %d bytes", bytes_sent);
}

// Wait for BUSY pin to go low
static void epaper_busy_wait(void)
{
    ESP_LOGI(TAG, "Waiting for BUSY pin...");
    uint32_t timeout = 20000; // 20 second timeout
    vTaskDelay(pdMS_TO_TICKS(5));
    
    while (gpio_get_level(EPAPER_BUSY_PIN) == 1) {
        vTaskDelay(pdMS_TO_TICKS(10));
        timeout -= 10;
        if (timeout == 0) {
            ESP_LOGW(TAG, "BUSY pin timeout");
            break;
        }
    }
    ESP_LOGI(TAG, "BUSY pin is low (ready)");
}

// Reset EPD
static void epaper_reset(void)
{
    ESP_LOGI(TAG, "Resetting EPD");
    gpio_set_level(EPAPER_RST_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(EPAPER_RST_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
}

// Set memory area
static void epaper_set_memory_area(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
  epaper_write_command(0x11); // set ram entry mode
  epaper_write_data(0x03);    // x increase, y increase : normal mode
  epaper_write_command(0x44);
  epaper_write_data(x / 8);
  epaper_write_data((x + w - 1) / 8);
  epaper_write_command(0x45);
  epaper_write_data(y % 256);
  epaper_write_data(y / 256);
  epaper_write_data((y + h - 1) % 256);
  epaper_write_data((y + h - 1) / 256);
  epaper_write_command(0x4e);
  epaper_write_data(x / 8);
  epaper_write_command(0x4f);
  epaper_write_data(y % 256);
  epaper_write_data(y / 256);
}


// Initialize EPD for 4.2 inch
static void epaper_init(void)
{
    epaper_reset();
    
    epaper_write_command(SW_RESET);
    epaper_busy_wait();
    
    // Driver output control
    epaper_write_command(DRIVER_OUTPUT_CONTROL);
    epaper_write_data((EPD_HEIGHT - 1) % 256);
    epaper_write_data(((EPD_HEIGHT - 1) / 256));
    epaper_write_data(0x00);  // GD = 0; SM = 0; TB = 0;
    
    // Booster soft start control
    epaper_write_command(BOOSTER_SOFT_START_CONTROL); //BorderWavefrom
    epaper_write_data(0x05);
    
    // read temperature register
    epaper_write_command(TEMPERATURE_SENSOR_READ);
    epaper_write_data(0x80);
    
    // Set display window
    epaper_set_memory_area(0, 0, EPD_WIDTH - 1, EPD_HEIGHT - 1);
    
    ESP_LOGI(TAG, "4.2 inch EPD initialized");
}

static void writeScreenBuffer(uint8_t command, uint8_t value)
{
    // to clear 0xff
    epaper_set_memory_area(0, 0, EPD_WIDTH - 1, EPD_HEIGHT - 1);
    epaper_write_command(command);
    gpio_set_level(EPAPER_CS_PIN, 0);  // CS start comms
    gpio_set_level(EPAPER_DC_PIN, 1);  // DC HIGH for data bytes
    for (uint32_t i = 0; i < ((uint32_t) EPD_WIDTH * (uint32_t) EPD_HEIGHT / 8) ; i++)
    {
        esp_err_t ret;
        spi_transaction_t t = {
            .length = 8,
            .tx_buffer = &value,
            .flags = 0,
        };
        ret = spi_device_polling_transmit(spi, &t);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to write command 0x%02X: %s", command, esp_err_to_name(ret));
        }
    }
    gpio_set_level(EPAPER_CS_PIN, 1);  // end comms


}

static void full_update()
{
    epaper_write_command(0x22);
    epaper_write_data(0xF7);
    epaper_write_command(0x20);
    epaper_busy_wait();

}
// Clear display (white) - using chunked transfer
static void epaper_clear(void)
{
    ESP_LOGI(TAG, "Clearing display");
    // const static int clear_value = 0x00;
    const static int black_off = 0xFF;
    const static int red_off = 0x00;
    writeScreenBuffer(0x24,black_off); //black
    writeScreenBuffer(0x26,red_off); // red
    full_update();
    epaper_busy_wait();
    ESP_LOGI(TAG, "Display cleared");
}

// Display image buffer using chunked transfer
static void epaper_display(const uint8_t *image_data)
{
    ESP_LOGI(TAG, "Displaying image (%d bytes)", EPD_BUFFER_SIZE);
    
    // Set full window
    epaper_set_memory_area(0, 0, EPD_WIDTH - 1, EPD_HEIGHT - 1);
    // writeScreenBuffer(0x26, 0xFF); // set red/white
    // writeScreenBuffer(0x24,0x00); // turn off black

    
    // // Send WRITE_RAM command
    // epaper_write_command(WRITE_RAM);
    
    // Send image data in chunks
    epaper_write_data_buffer_chunked(0x24, image_data, EPD_BUFFER_SIZE);
    // epaper_write_data_buffer_chunked(0x26, image_data, EPD_BUFFER_SIZE);
    
    // Update display
    full_update();    
    epaper_busy_wait();
    ESP_LOGI(TAG, "Image displayed");
}

// Create and display a simple test pattern
static void epaper_draw_test_pattern(void)
{
    ESP_LOGI(TAG, "Drawing test pattern");
    
    uint8_t *image_data = malloc(EPD_BUFFER_SIZE);
    if (image_data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for image");
        return;
    }
    
    // Fill with white
    memset(image_data, 0xFF, EPD_BUFFER_SIZE);
    
    // Draw a simple pattern: checkerboard
    for (uint16_t y = 0; y < EPD_HEIGHT; y++) {
        for (uint16_t x = 0; x < EPD_WIDTH; x++) {
            // Create 40x40 checkerboard pattern
            if (((x / 40) + (y / 40)) % 2 == 0) {
                uint32_t idx = (y * EPD_WIDTH + x) / 8;
                uint8_t bit = (y * EPD_WIDTH + x) % 8;
                image_data[idx] &= ~(1 << (7 - bit));  // Black
            }
        }
    }
    
    // Display the pattern
    epaper_display(image_data);
    free(image_data);
    
    ESP_LOGI(TAG, "Test pattern displayed");
}

static void epaper_set_pixel(uint8_t *buffer, uint16_t x, uint16_t y)
{
    if (x >= EPD_WIDTH || y >= EPD_HEIGHT) {
        return;
    }

    uint32_t byte_index = (y * EPD_WIDTH + x) / 8;
    uint8_t bit_mask = 1 << (7 - (x & 7));
    buffer[byte_index] &= ~bit_mask;
}

static const uint8_t font5x7[96][5] = {
    {0x00,0x00,0x00,0x00,0x00}, // space
    {0x00,0x00,0x5F,0x00,0x00}, // !
    {0x00,0x07,0x00,0x07,0x00}, // "
    {0x14,0x7F,0x14,0x7F,0x14}, // #
    {0x24,0x2A,0x7F,0x2A,0x12}, // $
    {0x23,0x13,0x08,0x64,0x62}, // %
    {0x36,0x49,0x55,0x22,0x50}, // &
    {0x00,0x05,0x03,0x00,0x00}, // '
    {0x00,0x1C,0x22,0x41,0x00}, // (
    {0x00,0x41,0x22,0x1C,0x00}, // )
    {0x14,0x08,0x3E,0x08,0x14}, // *
    {0x08,0x08,0x3E,0x08,0x08}, // +
    {0x00,0x50,0x30,0x00,0x00}, // ,
    {0x08,0x08,0x08,0x08,0x08}, // -
    {0x00,0x60,0x60,0x00,0x00}, // .
    {0x20,0x10,0x08,0x04,0x02}, // /
    {0x3E,0x51,0x49,0x45,0x3E}, // 0
    {0x00,0x42,0x7F,0x40,0x00}, // 1
    {0x42,0x61,0x51,0x49,0x46}, // 2
    {0x21,0x41,0x45,0x4B,0x31}, // 3
    {0x18,0x14,0x12,0x7F,0x10}, // 4
    {0x27,0x45,0x45,0x45,0x39}, // 5
    {0x3C,0x4A,0x49,0x49,0x30}, // 6
    {0x01,0x71,0x09,0x05,0x03}, // 7
    {0x36,0x49,0x49,0x49,0x36}, // 8
    {0x06,0x49,0x49,0x29,0x1E}, // 9
    {0x00,0x36,0x36,0x00,0x00}, // :
    {0x00,0x56,0x36,0x00,0x00}, // ;
    {0x08,0x14,0x22,0x41,0x00}, // <
    {0x14,0x14,0x14,0x14,0x14}, // =
    {0x00,0x41,0x22,0x14,0x08}, // >
    {0x02,0x01,0x51,0x09,0x06}, // ?
    {0x32,0x49,0x79,0x41,0x3E}, // @
    {0x7E,0x11,0x11,0x11,0x7E}, // A
    {0x7F,0x49,0x49,0x49,0x36}, // B
    {0x3E,0x41,0x41,0x41,0x22}, // C
    {0x7F,0x41,0x41,0x22,0x1C}, // D
    {0x7F,0x49,0x49,0x49,0x41}, // E
    {0x7F,0x09,0x09,0x09,0x01}, // F
    {0x3E,0x41,0x49,0x49,0x7A}, // G
    {0x7F,0x08,0x08,0x08,0x7F}, // H
    {0x00,0x41,0x7F,0x41,0x00}, // I
    {0x20,0x40,0x41,0x3F,0x01}, // J
    {0x7F,0x08,0x14,0x22,0x41}, // K
    {0x7F,0x40,0x40,0x40,0x40}, // L
    {0x7F,0x02,0x04,0x02,0x7F}, // M
    {0x7F,0x04,0x08,0x10,0x7F}, // N
    {0x3E,0x41,0x41,0x41,0x3E}, // O
    {0x7F,0x09,0x09,0x09,0x06}, // P
    {0x3E,0x41,0x51,0x21,0x5E}, // Q
    {0x7F,0x09,0x19,0x29,0x46}, // R
    {0x46,0x49,0x49,0x49,0x31}, // S
    {0x01,0x01,0x7F,0x01,0x01}, // T
    {0x3F,0x40,0x40,0x40,0x3F}, // U
    {0x1F,0x20,0x40,0x20,0x1F}, // V
    {0x3F,0x40,0x38,0x40,0x3F}, // W
    {0x63,0x14,0x08,0x14,0x63}, // X
    {0x07,0x08,0x70,0x08,0x07}, // Y
    {0x61,0x51,0x49,0x45,0x43}, // Z
    {0x00,0x7F,0x41,0x41,0x00}, // [
    {0x02,0x04,0x08,0x10,0x20}, // 
    {0x00,0x41,0x41,0x7F,0x00}, // ]
    {0x04,0x02,0x01,0x02,0x04}, // ^
    {0x40,0x40,0x40,0x40,0x40}, // _
    {0x00,0x03,0x07,0x08,0x00}, // `
    {0x20,0x54,0x54,0x54,0x78}, // a
    {0x7F,0x48,0x44,0x44,0x38}, // b
    {0x38,0x44,0x44,0x44,0x20}, // c
    {0x38,0x44,0x44,0x48,0x7F}, // d
    {0x38,0x54,0x54,0x54,0x18}, // e
    {0x08,0x7E,0x09,0x01,0x02}, // f
    {0x0C,0x52,0x52,0x52,0x3E}, // g
    {0x7F,0x08,0x04,0x04,0x78}, // h
    {0x00,0x44,0x7D,0x40,0x00}, // i
    {0x20,0x40,0x44,0x3D,0x00}, // j
    {0x7F,0x10,0x28,0x44,0x00}, // k
    {0x00,0x41,0x7F,0x40,0x00}, // l
    {0x7C,0x04,0x18,0x04,0x78}, // m
    {0x7C,0x08,0x04,0x04,0x78}, // n
    {0x38,0x44,0x44,0x44,0x38}, // o
    {0x7C,0x14,0x14,0x14,0x08}, // p
    {0x08,0x14,0x14,0x18,0x7C}, // q
    {0x7C,0x08,0x04,0x04,0x08}, // r
    {0x48,0x54,0x54,0x54,0x20}, // s
    {0x04,0x3F,0x44,0x40,0x20}, // t
    {0x3C,0x40,0x40,0x20,0x7C}, // u
    {0x1C,0x20,0x40,0x20,0x1C}, // v
    {0x3C,0x40,0x30,0x40,0x3C}, // w
    {0x44,0x28,0x10,0x28,0x44}, // x
    {0x0C,0x50,0x50,0x50,0x3C}, // y
    {0x44,0x64,0x54,0x4C,0x44}, // z
    {0x00,0x08,0x36,0x41,0x00}, // {
    {0x00,0x00,0x7F,0x00,0x00}, // |
    {0x00,0x41,0x36,0x08,0x00}, // }
    {0x08,0x04,0x08,0x10,0x08}, // ~
};

// Draw string on existing buffer (does not allocate or display)
static void epaper_draw_string_to_buffer(uint8_t *image_data, const char *text, uint16_t x, uint16_t y, font_size_t size)
{
    if (text == NULL) {
        ESP_LOGW(TAG, "No text provided to draw");
        return;
    }

    if (image_data == NULL) {
        ESP_LOGE(TAG, "Image buffer is NULL");
        return;
    }

    const uint16_t base_char_width = 5;
    const uint16_t base_char_height = 7;
    const uint16_t char_width = base_char_width * size;
    const uint16_t char_height = base_char_height * size;
    const uint16_t spacing_x = 1 * size;
    const uint16_t spacing_y = 2 * size;
    uint16_t cursor_x = x;
    uint16_t cursor_y = y;

    for (const char *p = text; *p != '\0'; p++) {
        char c = *p;
        if (c == '\n') {
            cursor_x = x;
            cursor_y += char_height + spacing_y;
            if (cursor_y + char_height >= EPD_HEIGHT) {
                break;
            }
            continue;
        }

        if ((uint8_t)c < 32 || (uint8_t)c > 127) {
            c = '?';
        }

        if (cursor_x + char_width >= EPD_WIDTH) {
            cursor_x = x;
            cursor_y += char_height + spacing_y;
            if (cursor_y + char_height >= EPD_HEIGHT) {
                break;
            }
        }

        const uint8_t *glyph = font5x7[(uint8_t)c - 32];
        for (uint16_t col = 0; col < base_char_width; col++) {
            uint8_t column = glyph[col];
            for (uint16_t row = 0; row < base_char_height; row++) {
                if (column & (1 << row)) {
                    for (uint16_t sx = 0; sx < size; sx++) {
                        for (uint16_t sy = 0; sy < size; sy++) {
                            epaper_set_pixel(image_data, cursor_x + col * size + sx, cursor_y + row * size + sy);
                        }
                    }
                }
            }
        }

        cursor_x += char_width + spacing_x;
    }
}

static void epaper_draw_string(const char *text, uint16_t x, uint16_t y, font_size_t size)
{
    if (text == NULL) {
        ESP_LOGW(TAG, "No text provided to draw");
        return;
    }

    uint8_t *image_data = malloc(EPD_BUFFER_SIZE);
    if (image_data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate buffer for text rendering");
        return;
    }
    memset(image_data, 0xFF, EPD_BUFFER_SIZE);

    const uint16_t base_char_width = 5;
    const uint16_t base_char_height = 7;
    const uint16_t char_width = base_char_width * size;
    const uint16_t char_height = base_char_height * size;
    const uint16_t spacing_x = 1 * size;
    const uint16_t spacing_y = 2 * size;
    uint16_t cursor_x = x;
    uint16_t cursor_y = y;

    for (const char *p = text; *p != '\0'; p++) {
        char c = *p;
        if (c == '\n') {
            cursor_x = x;
            cursor_y += char_height + spacing_y;
            if (cursor_y + char_height >= EPD_HEIGHT) {
                break;
            }
            continue;
        }

        if ((uint8_t)c < 32 || (uint8_t)c > 127) {
            c = '?';
        }

        if (cursor_x + char_width >= EPD_WIDTH) {
            cursor_x = x;
            cursor_y += char_height + spacing_y;
            if (cursor_y + char_height >= EPD_HEIGHT) {
                break;
            }
        }

        const uint8_t *glyph = font5x7[(uint8_t)c - 32];
        for (uint16_t col = 0; col < base_char_width; col++) {
            uint8_t column = glyph[col];
            for (uint16_t row = 0; row < base_char_height; row++) {
                if (column & (1 << row)) {
                    for (uint16_t sx = 0; sx < size; sx++) {
                        for (uint16_t sy = 0; sy < size; sy++) {
                            epaper_set_pixel(image_data, cursor_x + col * size + sx, cursor_y + row * size + sy);
                        }
                    }
                }
            }
        }

        cursor_x += char_width + spacing_x;
    }

    epaper_display(image_data);
    free(image_data);
}

// Put EPD to sleep
static void epaper_sleep(void)
{
    ESP_LOGI(TAG, "Putting EPD to sleep");
    epaper_write_command(DEEP_SLEEP_MODE);
    epaper_write_data(0x01);  // Enter deep sleep
    vTaskDelay(pdMS_TO_TICKS(100));
}

// Main task
void epaper_main_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Starting 4.2 inch EPD test");
    ESP_LOGI(TAG, "Display size: %dx%d, Buffer size: %d bytes", 
             EPD_WIDTH, EPD_HEIGHT, EPD_BUFFER_SIZE);
    
    // Initialize
    epaper_gpio_init();
    esp_err_t ret = epaper_spi_init();
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI initialization failed, aborting");
        vTaskDelete(NULL);
        return;
    }
    
    epaper_init();
    
    // Clear display first
    epaper_clear();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Create persistent buffer for multiple text strings
    uint8_t *screen_buffer = malloc(EPD_BUFFER_SIZE);
    if (screen_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate screen buffer");
        vTaskDelete(NULL);
        return;
    }
    
    // Fill buffer with white background
    memset(screen_buffer, 0xFF, EPD_BUFFER_SIZE);
    
    // Draw first string
    ESP_LOGI(TAG, "Drawing first text: Eyas Adriana wa Nur");
    epaper_draw_string_to_buffer(screen_buffer, "Prayer Times", 1, 1, FONT_SIZE_3X);
    
    // Draw second string
    ESP_LOGI(TAG, "Drawing second text: Eyas Adriana wa Nur second");
    epaper_draw_string_to_buffer(screen_buffer, "Fajr:", 0, 40, FONT_SIZE_2X);
    epaper_draw_string_to_buffer(screen_buffer, "Dhuhur:", 0, 80, FONT_SIZE_2X);
    epaper_draw_string_to_buffer(screen_buffer, "Asr:", 0, 120, FONT_SIZE_2X);
    epaper_draw_string_to_buffer(screen_buffer, "Ma3'rig:", 0, 160, FONT_SIZE_2X);
    epaper_draw_string_to_buffer(screen_buffer, "3isha2:", 0, 200, FONT_SIZE_2X);
    
    // Display the complete buffer once
    ESP_LOGI(TAG, "Displaying buffer with both strings");
    epaper_display(screen_buffer);
    free(screen_buffer);
    
    // Wait
    ESP_LOGI(TAG, "Waiting 15 seconds...");
    vTaskDelay(pdMS_TO_TICKS(15000));
    
    // Clear display again
    // epaper_clear();
    
    // // Wait before sleep
    // vTaskDelay(pdMS_TO_TICKS(5000));

    // epaper_clear();
    
    // Sleep
    epaper_sleep();
    
    ESP_LOGI(TAG, "Test complete");
    vTaskDelete(NULL);
}

// Main app
void app_main(void)
{
    // Start main task with larger stack
    xTaskCreate(epaper_main_task, "epaper_task", 12288, NULL, 5, NULL);
}