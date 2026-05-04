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

// ==================== Pin definitions ====================
#define EPAPER_RST_PIN     GPIO_NUM_17
#define EPAPER_DC_PIN      GPIO_NUM_16
#define EPAPER_CS_PIN      GPIO_NUM_5
#define EPAPER_BUSY_PIN    GPIO_NUM_4
#define EPAPER_MOSI_PIN    GPIO_NUM_23
#define EPAPER_SCLK_PIN    GPIO_NUM_18

// ==================== EPD geometry ====================
#define EPD_WIDTH          400
#define EPD_HEIGHT         300
#define EPD_BUFFER_SIZE    ((EPD_WIDTH * EPD_HEIGHT) / 8)   // 15000 bytes

// ==================== SPI settings ====================
#define SPI_CLOCK_SPEED_HZ     (4 * 1000 * 1000)   // 4 MHz (stable)
#define SPI_MAX_TRANSFER_SIZE  4096                // chunk size for DMA

// ==================== Display update modes ====================
#define UPDATE_MODE_FULL       0xF7   // full refresh, no ghosting
#define UPDATE_MODE_PARTIAL    0xF0   // fast partial refresh (some ghosting)
#define UPDATE_MODE_FASTEST    0xE0   // very fast, more ghosting

// ==================== E‑paper commands ====================
#define DRIVER_OUTPUT_CONTROL               0x01
#define BOOSTER_SOFT_START_CONTROL          0x3C
#define DEEP_SLEEP_MODE                     0x10
#define DATA_ENTRY_MODE_SETTING             0x11
#define SW_RESET                            0x12
#define TEMPERATURE_SENSOR_READ             0x18
#define DISPLAY_UPDATE_CONTROL_2            0x22
#define WRITE_RAM_BLACK                     0x24
#define WRITE_RAM_RED                       0x26
#define MASTER_ACTIVATION                   0x20
#define SET_RAM_X_ADDRESS                   0x44
#define SET_RAM_Y_ADDRESS                   0x45
#define SET_RAM_X_COUNTER                   0x4E
#define SET_RAM_Y_COUNTER                   0x4F

static const char *TAG = "Epaper";
static spi_device_handle_t spi;

// Global screen buffer (holds current display content)
static uint8_t *global_buffer = NULL;

// Font size enumeration
typedef enum {
    FONT_SIZE_1X = 1,
    FONT_SIZE_2X = 2,
    FONT_SIZE_3X = 3,
    FONT_SIZE_4X = 4
} font_size_t;

// ==================== SPI helpers (optimised) ====================

// Write a single command byte (DC low)
static void epaper_write_command(uint8_t cmd)
{
    spi_transaction_t t = { .length = 8, .tx_buffer = &cmd };
    gpio_set_level(EPAPER_CS_PIN, 0);
    gpio_set_level(EPAPER_DC_PIN, 0);
    spi_device_polling_transmit(spi, &t);
    gpio_set_level(EPAPER_CS_PIN, 1);
}

// Write a single data byte (DC high)
static void epaper_write_data(uint8_t data)
{
    spi_transaction_t t = { .length = 8, .tx_buffer = &data };
    gpio_set_level(EPAPER_CS_PIN, 0);
    gpio_set_level(EPAPER_DC_PIN, 1);
    spi_device_polling_transmit(spi, &t);
    gpio_set_level(EPAPER_CS_PIN, 1);
}

// Write command + data in one go (CS kept low, DC toggled)
static void epaper_write_cmd_data(uint8_t cmd, uint8_t data)
{
    spi_transaction_t t_cmd = { .length = 8, .tx_buffer = &cmd };
    spi_transaction_t t_data = { .length = 8, .tx_buffer = &data };
    gpio_set_level(EPAPER_CS_PIN, 0);
    gpio_set_level(EPAPER_DC_PIN, 0);
    spi_device_polling_transmit(spi, &t_cmd);
    gpio_set_level(EPAPER_DC_PIN, 1);
    spi_device_polling_transmit(spi, &t_data);
    gpio_set_level(EPAPER_CS_PIN, 1);
}

// Send a large buffer (DC must be high, CS low on entry)
static void epaper_write_data_buffer_raw(const uint8_t *data, uint32_t length)
{
    uint32_t sent = 0;
    while (sent < length) {
        uint32_t chunk = length - sent;
        if (chunk > SPI_MAX_TRANSFER_SIZE) chunk = SPI_MAX_TRANSFER_SIZE;
        spi_transaction_t t = {
            .length = chunk * 8,
            .tx_buffer = data + sent,
        };
        spi_device_polling_transmit(spi, &t);
        sent += chunk;
    }
}

// Send a repeated byte value (CS low, DC high on entry)
static void epaper_write_repeated_byte_raw(uint8_t value, uint32_t length)
{
    static uint8_t *tmp = NULL;
    static uint32_t tmp_size = 0;
    if (tmp_size < SPI_MAX_TRANSFER_SIZE) {
        if (tmp) free(tmp);
        tmp = malloc(SPI_MAX_TRANSFER_SIZE);
        if (!tmp) {
            ESP_LOGE(TAG, "Failed to allocate repeated byte buffer");
            return;
        }
        tmp_size = SPI_MAX_TRANSFER_SIZE;
        memset(tmp, value, tmp_size);
    } else if (tmp[0] != value) {
        memset(tmp, value, tmp_size);
    }

    uint32_t sent = 0;
    while (sent < length) {
        uint32_t chunk = length - sent;
        if (chunk > SPI_MAX_TRANSFER_SIZE) chunk = SPI_MAX_TRANSFER_SIZE;
        spi_transaction_t t = {
            .length = chunk * 8,
            .tx_buffer = tmp,
        };
        spi_device_polling_transmit(spi, &t);
        sent += chunk;
    }
}

// ==================== GPIO & SPI initialisation ====================

static void epaper_gpio_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << EPAPER_RST_PIN) | (1ULL << EPAPER_DC_PIN) | (1ULL << EPAPER_CS_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(EPAPER_RST_PIN, 1);
    gpio_set_level(EPAPER_DC_PIN, 0);
    gpio_set_level(EPAPER_CS_PIN, 1);

    io_conf.pin_bit_mask = (1ULL << EPAPER_BUSY_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    gpio_config(&io_conf);
}

static esp_err_t epaper_spi_init(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = EPAPER_MOSI_PIN,
        .miso_io_num = -1,
        .sclk_io_num = EPAPER_SCLK_PIN,
        .max_transfer_sz = SPI_MAX_TRANSFER_SIZE,
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SPI_CLOCK_SPEED_HZ,
        .mode = 0,
        .spics_io_num = EPAPER_CS_PIN,
        .queue_size = 7,
        .flags = SPI_DEVICE_HALFDUPLEX,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi));
    return ESP_OK;
}

// ==================== EPD primitives ====================

static void epaper_busy_wait(void)
{
    uint32_t timeout = 20000; // 20 seconds
    vTaskDelay(pdMS_TO_TICKS(5));
    while (gpio_get_level(EPAPER_BUSY_PIN) == 1) {
        vTaskDelay(pdMS_TO_TICKS(10));
        timeout -= 10;
        if (timeout == 0) {
            ESP_LOGW(TAG, "BUSY timeout");
            break;
        }
    }
}

static void epaper_reset(void)
{
    gpio_set_level(EPAPER_RST_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(EPAPER_RST_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
}

// Set the memory window (x_start, y_start) to (x_end, y_end) inclusive
static void epaper_set_memory_area(uint16_t x_start, uint16_t y_start,
                                   uint16_t x_end,   uint16_t y_end)
{
    epaper_write_cmd_data(DATA_ENTRY_MODE_SETTING, 0x03);   // x inc, y inc
    epaper_write_command(SET_RAM_X_ADDRESS);
    epaper_write_data(x_start / 8);
    epaper_write_data(x_end / 8);
    epaper_write_command(SET_RAM_Y_ADDRESS);
    epaper_write_data(y_start & 0xFF);
    epaper_write_data((y_start >> 8) & 0xFF);
    epaper_write_data(y_end & 0xFF);
    epaper_write_data((y_end >> 8) & 0xFF);
    epaper_write_cmd_data(SET_RAM_X_COUNTER, x_start / 8);
    epaper_write_command(SET_RAM_Y_COUNTER);
    epaper_write_data(y_start & 0xFF);
    epaper_write_data((y_start >> 8) & 0xFF);
}

static void epaper_init(void)
{
    epaper_reset();
    epaper_write_command(SW_RESET);
    epaper_busy_wait();

    // Driver output control
    epaper_write_command(DRIVER_OUTPUT_CONTROL);
    epaper_write_data((EPD_HEIGHT - 1) & 0xFF);
    epaper_write_data(((EPD_HEIGHT - 1) >> 8) & 0xFF);
    epaper_write_data(0x00);

    epaper_write_cmd_data(BOOSTER_SOFT_START_CONTROL, 0x05);
    epaper_write_cmd_data(TEMPERATURE_SENSOR_READ, 0x80);  // optional

    epaper_set_memory_area(0, 0, EPD_WIDTH - 1, EPD_HEIGHT - 1);
    ESP_LOGI(TAG, "EPD initialised");
}

// Trigger display update with a specific mode
static void epaper_update_display(uint8_t mode)
{
    epaper_write_cmd_data(DISPLAY_UPDATE_CONTROL_2, mode);
    epaper_write_command(MASTER_ACTIVATION);
    epaper_busy_wait();
}

// ==================== Full‑screen display operations ====================

static void epaper_clear(void)
{
    ESP_LOGI(TAG, "Full clear (white background)");
    epaper_set_memory_area(0, 0, EPD_WIDTH - 1, EPD_HEIGHT - 1);

    // Black channel: all white (0xFF)
    epaper_write_command(WRITE_RAM_BLACK);
    gpio_set_level(EPAPER_CS_PIN, 0);
    gpio_set_level(EPAPER_DC_PIN, 1);
    epaper_write_repeated_byte_raw(0xFF, EPD_BUFFER_SIZE);
    gpio_set_level(EPAPER_CS_PIN, 1);

    // Red channel: all off (0x00)
    epaper_write_command(WRITE_RAM_RED);
    gpio_set_level(EPAPER_CS_PIN, 0);
    gpio_set_level(EPAPER_DC_PIN, 1);
    epaper_write_repeated_byte_raw(0x00, EPD_BUFFER_SIZE);
    gpio_set_level(EPAPER_CS_PIN, 1);

    epaper_update_display(UPDATE_MODE_FULL);
}

// Display a full‑screen image (monochrome, black channel only)
static void epaper_display_full(const uint8_t *image_data)
{
    ESP_LOGI(TAG, "Full display update");
    epaper_set_memory_area(0, 0, EPD_WIDTH - 1, EPD_HEIGHT - 1);

    epaper_write_command(WRITE_RAM_BLACK);
    gpio_set_level(EPAPER_CS_PIN, 0);
    gpio_set_level(EPAPER_DC_PIN, 1);
    epaper_write_data_buffer_raw(image_data, EPD_BUFFER_SIZE);
    gpio_set_level(EPAPER_CS_PIN, 1);

    epaper_write_command(WRITE_RAM_RED);
    gpio_set_level(EPAPER_CS_PIN, 0);
    gpio_set_level(EPAPER_DC_PIN, 1);
    epaper_write_repeated_byte_raw(0x00, EPD_BUFFER_SIZE);
    gpio_set_level(EPAPER_CS_PIN, 1);

    epaper_update_display(UPDATE_MODE_FULL);
}

// ==================== Partial (fast) update ====================

// Perform a fast partial update on a rectangle.
// The rectangle is automatically aligned to byte boundaries (multiple of 8 horizontally).
static void epaper_partial_update(const uint8_t *image_data,
                                  uint16_t x_start, uint16_t y_start,
                                  uint16_t x_end,   uint16_t y_end)
{
    // Align x_start down and x_end up to multiples of 8
    uint16_t x_start_byte = (x_start / 8) * 8;
    uint16_t x_end_byte   = ((x_end + 7) / 8) * 8 - 1;
    if (x_end_byte >= EPD_WIDTH) x_end_byte = EPD_WIDTH - 1;

    uint16_t cols = (x_end_byte - x_start_byte + 1) / 8;   // bytes per row
    uint16_t rows = y_end - y_start + 1;
    uint32_t total_bytes = cols * rows;

    // Extract the sub‑image from the full buffer
    uint8_t *subbuffer = malloc(total_bytes);
    if (!subbuffer) {
        ESP_LOGE(TAG, "Cannot allocate subbuffer for partial update");
        return;
    }
    for (uint16_t y = y_start; y <= y_end; y++) {
        uint32_t src_offset = y * (EPD_WIDTH / 8) + (x_start_byte / 8);
        uint32_t dst_offset = (y - y_start) * cols;
        memcpy(subbuffer + dst_offset, image_data + src_offset, cols);
    }

    // Set memory window to the aligned rectangle
    epaper_set_memory_area(x_start_byte, y_start, x_end_byte, y_end);

    // Send black channel data
    epaper_write_command(WRITE_RAM_BLACK);
    gpio_set_level(EPAPER_CS_PIN, 0);
    gpio_set_level(EPAPER_DC_PIN, 1);
    epaper_write_data_buffer_raw(subbuffer, total_bytes);
    gpio_set_level(EPAPER_CS_PIN, 1);

    // Red channel – all zeros (red not used in partial updates)
    epaper_write_command(WRITE_RAM_RED);
    gpio_set_level(EPAPER_CS_PIN, 0);
    gpio_set_level(EPAPER_DC_PIN, 1);
    epaper_write_repeated_byte_raw(0x00, total_bytes);
    gpio_set_level(EPAPER_CS_PIN, 1);

    free(subbuffer);

    // Fast partial refresh
    epaper_update_display(UPDATE_MODE_PARTIAL);
}

// ==================== Font and drawing routines ====================

// Font 5x7 – full ASCII 32..127 (same as original)
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

// Set a single pixel to black (direct bit manipulation, inline)
static inline void set_pixel_black(uint8_t *buffer, uint16_t x, uint16_t y)
{
    if (x >= EPD_WIDTH || y >= EPD_HEIGHT) return;
    uint32_t idx = (y * EPD_WIDTH + x) >> 3;
    uint8_t bit = 7 - (x & 7);
    buffer[idx] &= ~(1 << bit);
}

// Draw a scaled character into a buffer
static void draw_char_scaled(uint8_t *buffer, char c, uint16_t x, uint16_t y, font_size_t size)
{
    if ((uint8_t)c < 32 || (uint8_t)c > 127) c = '?';
    const uint8_t *glyph = font5x7[(uint8_t)c - 32];
    uint16_t scale = (uint16_t)size;
    for (uint16_t col = 0; col < 5; col++) {
        uint8_t col_bits = glyph[col];
        if (col_bits == 0) continue;
        for (uint16_t row = 0; row < 7; row++) {
            if (col_bits & (1 << row)) {
                uint16_t px = x + col * scale;
                uint16_t py = y + row * scale;
                for (uint16_t sx = 0; sx < scale; sx++) {
                    for (uint16_t sy = 0; sy < scale; sy++) {
                        set_pixel_black(buffer, px + sx, py + sy);
                    }
                }
            }
        }
    }
}

// Draw a string into a buffer (no display)
void epaper_draw_string_to_buffer(uint8_t *image_data, const char *text,
                                  uint16_t x, uint16_t y, font_size_t size)
{
    if (!image_data || !text) return;
    uint16_t cursor_x = x, cursor_y = y;
    uint16_t char_width = 5 * size;
    uint16_t char_height = 7 * size;
    uint16_t spacing_x = 1 * size;
    uint16_t spacing_y = 2 * size;

    for (const char *p = text; *p; p++) {
        char c = *p;
        if (c == '\n') {
            cursor_x = x;
            cursor_y += char_height + spacing_y;
            if (cursor_y + char_height >= EPD_HEIGHT) break;
            continue;
        }
        if (cursor_x + char_width >= EPD_WIDTH) {
            cursor_x = x;
            cursor_y += char_height + spacing_y;
            if (cursor_y + char_height >= EPD_HEIGHT) break;
        }
        draw_char_scaled(image_data, c, cursor_x, cursor_y, size);
        cursor_x += char_width + spacing_x;
    }
}

// Allocate a buffer, draw string, and display (full refresh)
void epaper_draw_string(const char *text, uint16_t x, uint16_t y, font_size_t size)
{
    if (!text) return;
    uint8_t *buffer = malloc(EPD_BUFFER_SIZE);
    if (!buffer) {
        ESP_LOGE(TAG, "Cannot allocate buffer for draw_string");
        return;
    }
    memset(buffer, 0xFF, EPD_BUFFER_SIZE);
    epaper_draw_string_to_buffer(buffer, text, x, y, size);
    epaper_display_full(buffer);
    free(buffer);
}

// Draw string using a fast partial update (requires global buffer)
void epaper_draw_string_partial(const char *text, uint16_t x, uint16_t y, font_size_t size)
{
    if (!text || !global_buffer) return;

    // Estimate text bounding box (rough, but safe)
    uint16_t text_width = strlen(text) * (5 * size + 1 * size);
    uint16_t text_height = 7 * size;
    uint16_t x_end = x + text_width;
    uint16_t y_end = y + text_height;
    if (x_end >= EPD_WIDTH) x_end = EPD_WIDTH - 1;
    if (y_end >= EPD_HEIGHT) y_end = EPD_HEIGHT - 1;

    // Draw into global buffer
    epaper_draw_string_to_buffer(global_buffer, text, x, y, size);

    // Partial update on that rectangle
    epaper_partial_update(global_buffer, x, y, x_end, y_end);
}

// Put EPD into deep sleep
static void epaper_sleep(void)
{
    ESP_LOGI(TAG, "Deep sleep");
    epaper_write_cmd_data(DEEP_SLEEP_MODE, 0x01);
    vTaskDelay(pdMS_TO_TICKS(100));
}

// ==================== Main demonstration task ====================

void epaper_main_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Starting EPD420_BWR driver");
    epaper_gpio_init();
    epaper_spi_init();
    epaper_init();
    epaper_clear();
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Allocate global buffer for persistent content
    global_buffer = malloc(EPD_BUFFER_SIZE);
    if (!global_buffer) {
        ESP_LOGE(TAG, "Failed to allocate global buffer");
        vTaskDelete(NULL);
        return;
    }
    memset(global_buffer, 0xFF, EPD_BUFFER_SIZE);

    // Draw initial screen (full refresh)
    epaper_draw_string_to_buffer(global_buffer, "Prayer Times", 1, 1, FONT_SIZE_3X);
    epaper_draw_string_to_buffer(global_buffer, "Fajr:",    0, 40, FONT_SIZE_2X);
    epaper_draw_string_to_buffer(global_buffer, "Dhuhur:",  0, 80, FONT_SIZE_2X);
    epaper_draw_string_to_buffer(global_buffer, "Asr:",     0, 120, FONT_SIZE_2X);
    epaper_draw_string_to_buffer(global_buffer, "Ma'rib:",  0, 160, FONT_SIZE_2X);
    epaper_draw_string_to_buffer(global_buffer, "3isha2:",  0, 200, FONT_SIZE_2X);
    epaper_display_full(global_buffer);

    // Example: fast update of a dynamic value (e.g., time)
    int counter = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));   // update every 10 seconds
        char time_str[20];
        snprintf(time_str, sizeof(time_str), "%02d:%02d", (counter / 6) % 24, (counter * 10) % 60);
        ESP_LOGI(TAG ,"time_str = %s",time_str);

        // Overwrite the same area with new text
        epaper_draw_string_partial(time_str, 240, 45, FONT_SIZE_2X);
        counter++;
        if (counter % 30 == 0) {
            // After many partial updates, do a full refresh to clear ghosting
            epaper_display_full(global_buffer);
        }
    }

    // (Never reached, but if you need to sleep at the end:)
    // epaper_sleep();
    // free(global_buffer);
    // vTaskDelete(NULL);
}

void app_main(void)
{
    xTaskCreate(epaper_main_task, "epaper_task", 12288, NULL, 5, NULL);
}