#include "esp_log.h"

#include "STUSB4500.h"
#include "i2c_bitaxe.h"

#define STUSB4500_I2CADDR_DEFAULT 0x28

static i2c_master_dev_handle_t emc2101_dev_handle;

static const char * TAG = "STUSB4500";

esp_err_t STUSB4500_init()
{
    if (i2c_bitaxe_add_device(STUSB4500_I2CADDR_DEFAULT, &emc2101_dev_handle, TAG) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device");
        return ESP_FAIL;
    }
    return 0;
}