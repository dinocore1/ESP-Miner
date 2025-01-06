#include "esp_log.h"
#include <esp_check.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "STUSB4500.h"
#include "i2c_bitaxe.h"

#include "STUSB4500_def.h"

#define STUSB4500_I2CADDR_DEFAULT 0x28
#define ALERT_PIN 22

static i2c_master_dev_handle_t stusb4500_dev_handle;

static const char * TAG = "STUSB4500";

static SemaphoreHandle_t alertSemaphore = NULL;

static void alert_isr_handler(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(alertSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

static void set_PDOSnk_count(uint8_t const count)
{
    i2c_bitaxe_register_write_byte(stusb4500_dev_handle, REG_DPM_PDO_NUM, count);
}

esp_err_t STUSB4500_init()
{
    if (i2c_bitaxe_add_device(STUSB4500_I2CADDR_DEFAULT, &stusb4500_dev_handle, TAG) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device");
        return ESP_FAIL;
    }

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ALERT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&io_conf);

    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(ALERT_PIN, alert_isr_handler, NULL), TAG, "adding ISR handler");

    alertSemaphore = xSemaphoreCreateCounting(10, 0);

    set_PDOSnk_count(1);

    // clear all interrupts by reading all 10 registers from 0x0d to 0x16
    for (uint8_t reg_addr = 0x0d; reg_addr < 0x16; reg_addr++) {
        uint8_t read_buf[1];
        i2c_bitaxe_register_read(stusb4500_dev_handle, reg_addr, read_buf, 1);
    }





    return ESP_OK;
}