#include <stdlib.h>

#include <driver/gpio.h>
#include <esp_check.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <async.h>

#include "STUSB4500.h"
#include "i2c_bitaxe.h"

#include "STUSB4500_def.h"

#define max(a, b) ((a) > (b) ? (a) : (b))
#define min(a, b) ((a) < (b) ? (a) : (b))

#define STUSB4500_I2CADDR_DEFAULT 0x28
#define ALERT_PIN 14

#define TASK_NOTIFY_ISR_ALART (1 << 0)
#define TASK_NOTIFY_OTHER (1 << 1)

#define LE16(addr) (((uint16_t) (*((uint8_t *) (addr)))) + (((uint16_t) (*(((uint8_t *) (addr)) + 1))) << 8))

#define LE32(addr)                                                                                                                 \
    ((((uint32_t) (*(((uint8_t *) (addr)) + 0))) + (((uint32_t) (*(((uint8_t *) (addr)) + 1))) << 8) +                             \
      (((uint32_t) (*(((uint8_t *) (addr)) + 2))) << 16) + (((uint32_t) (*(((uint8_t *) (addr)) + 3))) << 24)))

static i2c_master_dev_handle_t stusb4500_dev_handle;

static const char * TAG = "STUSB4500";

static TaskHandle_t taskHandle = NULL;

static STUSB_GEN1S_ALERT_STATUS_RegTypeDef status;
static STUSB_GEN1S_PRT_STATUS_RegTypeDef PRT_status;
static uint8_t sNumSourcePDOsAvailable;
static USB_PD_SRC_PDOTypeDef sSourcePDOs[7];

static void stusb4500_task(void * params);

static void alert_isr_handler(void * arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xTaskNotifyFromISR(taskHandle, TASK_NOTIFY_ISR_ALART, eSetBits, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void stusb4500_setPDOCount(uint8_t const count)
{
    i2c_bitaxe_register_write_byte(stusb4500_dev_handle, REG_DPM_PDO_NUM, count);
}

static void stusb4500_softReset()
{
    // SOFT_RESET
    i2c_bitaxe_register_write_byte(stusb4500_dev_handle, REG_TX_HEADER_LOW, 0x0D);

    // SEND_COMMAND
    i2c_bitaxe_register_write_byte(stusb4500_dev_handle, REG_PD_COMMAND_CTRL, 0x26);
}

static void stusb4500_writePDO(uint8_t pdo_num, USB_PD_SRC_PDOTypeDef pdo)
{
    uint8_t buffer[5];

    buffer[0] = 0x85 + (pdo_num * 4);
    buffer[1] = (pdo.d32 >> 0) & 0xFF;
    buffer[2] = (pdo.d32 >> 8) & 0xFF;
    buffer[3] = (pdo.d32 >> 16) & 0xFF;
    buffer[4] = (pdo.d32 >> 24) & 0xFF;

    if (i2c_bitaxe_register_write_bytes(stusb4500_dev_handle, buffer, 5) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write PDO");
    }
}

static USB_PD_SRC_PDOTypeDef stusb4500_createFixedPDO(uint16_t milli_volts, uint16_t milli_amps)
{
    USB_PD_SRC_PDOTypeDef retval.d32 = 0;

    retval.fix.Voltage = milli_volts / 50;
    retval.fix.Max_Operating_Current = milli_amps / 10;

    return retval;
}

struct src_pdo_sort
{
    uint8_t idx;
    uint16_t milli_volts;
    uint16_t milli_amps;
};

int pdo_sort_cmp(const void * a, const void * b)
{
    int ret;
    struct src_pdo_sort * pdoa = (struct src_pdo_sort *) a;
    struct src_pdo_sort * pdob = (struct src_pdo_sort *) b;

    int a_watts = (int) pdoa->milli_volts * (int) pdoa->milli_amps;
    int b_watts = (int) pdob->milli_volts * (int) pdob->milli_amps;

    ret = b_watts - a_watts;
    if (ret == 0) {
        // sort by secondary key: lowest voltage
        ret = pdoa->milli_volts - pdob->milli_volts;
    }

    return ret;
}

static struct src_pdo_sort sort_pdo()
{
    struct src_pdo_sort my_list[10];

    for (int i = 0; i < sNumSourcePDOsAvailable; i++) {
        uint16_t milli_volts;
        uint16_t milli_amps;

        switch (sSourcePDOs[i].fix.FixedSupply) {
        case 0:
            // fixed supply
            milli_volts = sSourcePDOs[i].fix.Voltage * 50;
            milli_amps = sSourcePDOs[i].fix.Max_Operating_Current * 10;
            break;

        case 1:
            // Variable supply
            milli_volts = min(sSourcePDOs[i].var.Min_Voltage, sSourcePDOs[i].var.Max_Voltage) * 50;
            milli_amps = sSourcePDOs[i].var.Operating_Current * 10;
            break;
        }
        my_list[i].idx = i;
        my_list[i].milli_volts = milli_volts;
    }

    qsort(my_list, sNumSourcePDOsAvailable, sizeof(struct src_pdo_sort), pdo_sort_cmp);

    return my_list[0];
}

static void read_status_registers()
{
    uint8_t scratch[40];

    ESP_GOTO_ON_ERROR(i2c_bitaxe_register_read(stusb4500_dev_handle, REG_ALERT_STATUS_1, scratch, 2), exit, TAG,
                      "reading status reg");
    status.d8 = scratch[0] & ~scratch[1];

    ESP_LOGD(TAG, "ALERT_STATUS: 0x%x", status.d8);

    if (status.b.PRT_STATUS_AL) {

        ESP_GOTO_ON_ERROR(i2c_bitaxe_register_read(stusb4500_dev_handle, REG_PRT_STATUS, &PRT_status.d8, 1), exit, TAG,
                          "reading PRT_status reg");

        ESP_LOGD(TAG, "PRT_STATUS: 0x%x", PRT_status.d8);

        if (PRT_status.b.MSG_RECEIVED) {
            USBPD_MsgHeader_TypeDef header;

            ESP_GOTO_ON_ERROR(i2c_bitaxe_register_read(stusb4500_dev_handle, REG_RX_HEADER, scratch, 2), exit, TAG,
                              "reading RX_HEADER");
            header.d16 = LE16(&scratch[0]);

            ESP_LOGD(TAG, "RX_HEADER: 0x%x", header.d16);

            if (header.b.NumberOfDataObjects > 0) {
                switch (header.b.MessageType) {
                case 0x01:
                    ESP_GOTO_ON_ERROR(
                        i2c_bitaxe_register_read(stusb4500_dev_handle, REG_RX_DATA_OBJ, scratch, header.b.NumberOfDataObjects * 4),
                        exit, TAG, "read RX_DATA_OBJ");

                    for (int i = 0; i < header.b.NumberOfDataObjects; i++) {
                        sSourcePDOs[i].d32 = LE32(&scratch[i * 4]);
                        ESP_LOGD(TAG, "RX_DATA_OBJ[%d]: 0x%x", i, sSourcePDOs[i].d32);
                    }
                    sNumSourcePDOsAvailable = header.b.NumberOfDataObjects;

                    break;
                default:
                    break;
                }
            }
        }
    }

exit:
}

static async run(struct async* pt)
{
    struct src_pdo_sort pdo;
    USB_PD_SRC_PDOTypeDef pdo_usbpd;

    async_begin(pt);

    await(sNumSourcePDOsAvailable > 0);

    pdo = sort_pdo();
    pdo_usbpd = stusb4500_createFixedPDO(pdo.milli_volts, pdo.milli_amps);
    stusb4500_writePDO(1, pdo_usbpd);
    stusb4500_setPDOCount(2);
    stusb4500_softReset();

    async_end;
}

static void stusb4500_task(void * params)
{
    const TickType_t xBlockTime = pdMS_TO_TICKS(500);
    uint32_t ulNotifiedValue;
    esp_err_t ret;
    struct async run_task;

    async_init(&run_task);

    for (;;) {

        run(&run_task);

        ulNotifiedValue = ulTaskNotifyTake(pdTRUE, xBlockTime);

        if (ulNotifiedValue & TASK_NOTIFY_ISR_ALART) {
            read_status_registers();
        }

    }
}

esp_err_t STUSB4500_init()
{
    uint8_t device_id;

    sNumSourcePDOsAvailable = 0;

    ESP_LOGI(TAG, "Initializing STUSB4500");
    if (i2c_bitaxe_add_device(STUSB4500_I2CADDR_DEFAULT, &stusb4500_dev_handle, TAG) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device");
        return ESP_FAIL;
    }

    ESP_RETURN_ON_ERROR(i2c_bitaxe_register_read(stusb4500_dev_handle, REG_DEVICE_ID, &device_id, 1), TAG, "reading device id");
    ESP_LOGI(TAG, "device id: 0x%x", device_id);
    ESP_RETURN_ON_FALSE(device_id == 0x25, ESP_FAIL, TAG, "device id mismatch expecting 0x25");

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ALERT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&io_conf);

    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(ALERT_PIN, alert_isr_handler, NULL), TAG, "adding ISR handler");

    xTaskCreate(stusb4500_task, TAG, 4096, NULL, 10, &taskHandle);

    stusb4500_createFixedPDO(5000, 500);
    stusb4500_setPDOCount(1);

    // clear all interrupts by reading all 10 registers from 0x0d to 0x16
    for (uint8_t reg_addr = 0x0d; reg_addr < 0x16; reg_addr++) {
        uint8_t read_buf[1];
        i2c_bitaxe_register_read(stusb4500_dev_handle, reg_addr, read_buf, 1);
    }

    return ESP_OK;
}
