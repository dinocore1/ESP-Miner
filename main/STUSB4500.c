#include <stdbool.h>
#include <stdlib.h>

#include <driver/gpio.h>
#include <esp_check.h>
#include <esp_log.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <async.h>

#include "STUSB4500.h"
#include "STUSB4500_register.h"
#include "STUSB4500_def.h"

#include "i2c_bitaxe.h"

#define max(a, b) ((a) > (b) ? (a) : (b))
#define min(a, b) ((a) < (b) ? (a) : (b))

#define STUSB4500_I2CADDR_DEFAULT 0x28
#define ALERT_PIN 14

#define EVAL_DEVICE_ID 0x21 // in device ID reg (0x2F)
#define PROD_DEVICE_ID 0x25 //

// convert value at addr to little-endian (16-bit)
#define LE_u16(addr) (((((uint16_t) (*(((uint8_t *) (addr)) + 1)))) + (((uint16_t) (*(((uint8_t *) (addr)) + 0))) << 8)))

// convert value at addr to little-endian (32-bit)
#define LE_u32(addr)                                                                                                               \
    (((((uint32_t) (*(((uint8_t *) (addr)) + 3)))) + (((uint32_t) (*(((uint8_t *) (addr)) + 2))) << 8) +                           \
      (((uint32_t) (*(((uint8_t *) (addr)) + 1))) << 16) + (((uint32_t) (*(((uint8_t *) (addr)) + 0))) << 24)))

#define CABLE_CONNECTED(c) ((CC1Connected == (c)) || (CC2Connected == (c)))

static i2c_master_dev_handle_t stusb4500_dev_handle;
static uint16_t _srcCapRequestMax;
static USBPDStatus_t _status;
static USBPDStateMachine_t _state;
static PDO_t _snkRDO;
static PDO_t _snkPDO[NVM_SNK_PDO_MAX];
static PDO_t _srcPDO[NVM_SRC_PDO_MAX];

static const char * TAG = "STUSB4500";

static TaskHandle_t taskHandle = NULL;

static void stusb4500_task(void * params);

static volatile uint16_t alert_count = 0;

static volatile uint8_t is_power_ready = 0;

void stusb4500_setPDOSnkCount(uint8_t const count)
{
    i2c_bitaxe_register_write_byte(stusb4500_dev_handle, DPM_PDO_NUMB, count);
}

bool stusb4500_ready(void)
{
    uint8_t deviceID;
    i2c_bitaxe_register_read(stusb4500_dev_handle, REG_DEVICE_ID, &deviceID, 1);
    return (EVAL_DEVICE_ID == deviceID) || (PROD_DEVICE_ID == deviceID);
}

void stusb4500_waitUntilReady(void)
{
    while (!stusb4500_ready()) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void stusb4500_updatePDOSnk(void)
{
#define BUFF_SZ NVM_SNK_PDO_MAX * sizeof(USB_PD_SNK_PDO_TypeDef)
    uint8_t pdo[BUFF_SZ];
    uint8_t pdoCount;

    stusb4500_waitUntilReady();

    i2c_bitaxe_register_read(stusb4500_dev_handle, DPM_PDO_NUMB, &pdoCount, 1);

    i2c_bitaxe_register_read(stusb4500_dev_handle, DPM_SNK_PDO1, pdo, BUFF_SZ);

    _status.pdoSnkCount = pdoCount;
    for (uint8_t i = 0, j = 0; i < NVM_SNK_PDO_MAX; ++i, j += 4) {
        if (i < pdoCount) {
            _status.pdoSnk[i].d32 = LE_u32(&pdo[j]);
            PDO_init(&_snkPDO[i], i + 1, _status.pdoSnk[i].fix.Voltage * 50U, _status.pdoSnk[i].fix.Operational_Current * 10U, 0);
        } else {
            _status.pdoSnk[i].d32 = 0U;
            PDO_init_zero(&_snkPDO[i]);
        }
    }
#undef BUFF_SZ
}

void stusb4500_updatePDOSrc()
{
    stusb4500_waitUntilReady();

    // DELAY(TLOAD_REG_INIT_MS);

    uint8_t maxRequests = _srcCapRequestMax;
    uint8_t request = 0U;

    ++(_state.srcPDORequesting);

    while ((0U != _state.srcPDORequesting) && (request < maxRequests)) {
        if (!stusb4500_sendPDCableReset()) {
            break;
        }
        processAlerts();
        ++request;
    }

}

void stusb4500_updateRDOSnk()
{
    STUSB_GEN1S_RDO_REG_STATUS_RegTypeDef rdo;
    i2c_bitaxe_register_read(stusb4500_dev_handle, RDO_REG_STATUS, (uint8_t *) &rdo, sizeof(rdo));

    if (rdo.b.Object_Pos > 0) {

        _snkRDO.number = rdo.b.Object_Pos;
        _snkRDO.current_mA = rdo.b.OperatingCurrent * 10U;
        _snkRDO.maxCurrent_mA = rdo.b.MaxCurrent * 10U;

        if (_snkRDO.number <= _status.pdoSnkCount) {
            _snkRDO.voltage_mV = _status.pdoSnk[_snkRDO.number - 1U].fix.Voltage * 50U;
        }
    } else {
        PDO_init_zero(&_snkRDO);
    }
}

/****
 * clear all pending alerts by reading the status registers.
 ****/
void stusb4500_clearAlerts(bool const unmask)
{
    // clear alert status
    uint8_t alertStatus[12];
    i2c_bitaxe_register_read(stusb4500_dev_handle, ALERT_STATUS_1, alertStatus, 12);

    STUSB_GEN1S_ALERT_STATUS_MASK_RegTypeDef alertMask;
    if (unmask) {

        // set interrupts to unmask
        alertMask.d8 = 0xFF;

        // alertMask.b.PHY_STATUS_AL_MASK          = 0U;
        alertMask.b.PRT_STATUS_AL_MASK = 0U;
        alertMask.b.PD_TYPEC_STATUS_AL_MASK = 0U;
        // alertMask.b.HW_FAULT_STATUS_AL_MASK     = 0U;
        alertMask.b.MONITORING_STATUS_AL_MASK = 0U;
        alertMask.b.CC_DETECTION_STATUS_AL_MASK = 0U;
        alertMask.b.HARD_RESET_AL_MASK = 0U;

        // unmask the above alarms
        i2c_bitaxe_register_write_byte(stusb4500_dev_handle, ALERT_STATUS_MASK, alertMask.d8);
    }
}

void stusb4500_updatePrtStatus(void)
{
    uint8_t prtStatus[10];
    i2c_bitaxe_register_read(stusb4500_dev_handle, REG_PORT_STATUS, prtStatus, 10U);

    _status.ccDetectionStatus.d8 = prtStatus[1];
    _status.ccStatus.d8 = prtStatus[3];
    _status.monitoringStatus.d8 = prtStatus[3];
    _status.hwFaultStatus.d8 = prtStatus[6];
}

void stusb4500_clearPDOSrc(void)
{
    _status.pdoSrcCount = 0U;
    for (uint8_t i = 0; i < NVM_SRC_PDO_MAX; ++i) {
        _status.pdoSrc[i].d32 = 0U;
        PDO_init_zero(&_srcPDO[i]);
    }
}

CableStatus_t stusb4500_cableStatus(void)
{
    uint8_t portStatus;
    uint8_t typeCStatus;

    if (i2c_bitaxe_register_read(stusb4500_dev_handle, REG_PORT_STATUS, &portStatus, 1U) != ESP_OK) {
        return NONE;
    }

    if (VALUE_ATTACHED == (portStatus & STUSBMASK_ATTACHED_STATUS)) {
        if (i2c_bitaxe_register_read(stusb4500_dev_handle, REG_TYPE_C_STATUS, &typeCStatus, 1U) != ESP_OK) {
            return NONE;
        }

        if (0U == (typeCStatus & MASK_REVERSE)) {
            return CC1Connected;
        } else {
            return CC2Connected;
        }
    } else {
        return NotConnected;
    }
}

static void alert_isr_handler(void * arg)
{
    alert_count++;
    // BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // xTaskNotifyFromISR(taskHandle, TASK_NOTIFY_ISR_ALART, eSetBits, &xHigherPriorityTaskWoken);
    // portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void STUSB4500_wait_for_power_ready()
{
    while (is_power_ready == 0) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// static void stusb4500_sw_reset()
// {
//     uint8_t scratch[12];
//     int i;

//     i2c_bitaxe_register_write_byte(stusb4500_dev_handle, REG_STUSB_GEN1S_RESET_CTRL, 1);

//     for (i = 0; i < 2; i++) {
//         i2c_bitaxe_register_read(stusb4500_dev_handle, REG_DEVICE_ID, scratch, 1);
//     }

//     for (i = 0; i < 12; i++) {
//         i2c_bitaxe_register_read(stusb4500_dev_handle, REG_ALERT_STATUS_1 + 1, scratch, 1);
//     }

//     vTaskDelay(pdMS_TO_TICKS(27));

//     i2c_bitaxe_register_write_byte(stusb4500_dev_handle, REG_STUSB_GEN1S_RESET_CTRL, 0);
// }

/**
 * prints the current negociated power contract
 */
// static void print_power_contract()
// {
//     STUSB_GEN1S_RDO_REG_STATUS_RegTypeDef Nego_RDO;
//     i2c_bitaxe_register_read(stusb4500_dev_handle, REG_RDO_STATUS, (uint8_t *) &Nego_RDO.d32, 4);
//     ESP_LOGD(TAG, "RDO: 0x%lx", Nego_RDO.d32);

//     if (Nego_RDO.d32 != 0) {
//         ESP_LOGI(TAG, "Requested position: PDO %d", Nego_RDO.b.Object_Pos);

//         int OpCurrent_mA = Nego_RDO.b.OperatingCurrent * 10;
//         int MaxCurrent_mA = Nego_RDO.b.MaxCurrent * 10;
//         ESP_LOGI(TAG, "Operating Current: %d mA , Max Current: %d mA", OpCurrent_mA, MaxCurrent_mA);
//         ESP_LOGI(TAG, "USB Com capable: %d, Capability Mismatch: %d", Nego_RDO.b.UsbComCap, Nego_RDO.b.CapaMismatch);
//     } else {
//         ESP_LOGI(TAG, "No explicit Contract yet");
//     }

//     uint8_t value;
//     i2c_bitaxe_register_read(stusb4500_dev_handle, 0x21, &value, 1);
//     uint16_t milli_volts = value * 100;
//     ESP_LOGI(TAG, "Voltage requested: %d mV", milli_volts);
// }

// static void stusb4500_softReset()
// {
//     ESP_LOGD(TAG, "performing soft reset");

//     // SOFT_RESET
//     i2c_bitaxe_register_write_byte(stusb4500_dev_handle, REG_TX_HEADER_LOW, 0x0D);
//     i2c_bitaxe_register_write_byte(stusb4500_dev_handle, REG_TX_HEADER_HIGH, 0x00);

//     // SEND_COMMAND
//     i2c_bitaxe_register_write_byte(stusb4500_dev_handle, REG_PD_COMMAND_CTRL, 0x26);
// }

bool stusb4500_sendPDCableReset()
{
    CableStatus_t cable = stusb4500_cableStatus();
    if (!CABLE_CONNECTED(cable)) {
        return false;
    }

    // send PD message "soft reset" to source by setting TX header (0x51) to 0x0D,
    // and set PD command (0x1A) to 0x26.

    if (i2c_bitaxe_register_write_byte(stusb4500_dev_handle, TX_HEADER, 0x0D) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write TX_HEADER");
        return false;
    }

    if (i2c_bitaxe_register_write_byte(stusb4500_dev_handle, STUSB_GEN1S_CMD_CTRL, 0x26) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write PD_COMMAND_CTRL");
        return false;
    }

    return true;
}

// static void stusb4500_writePDO(uint8_t pdo_num, USB_PD_SRC_PDOTypeDef pdo)
// {
//     uint8_t buffer[5];

//     buffer[0] = 0x85 + (pdo_num * 4);
//     buffer[1] = (pdo.d32 >> 0) & 0xFF;
//     buffer[2] = (pdo.d32 >> 8) & 0xFF;
//     buffer[3] = (pdo.d32 >> 16) & 0xFF;
//     buffer[4] = (pdo.d32 >> 24) & 0xFF;

//     if (i2c_bitaxe_register_write_bytes(stusb4500_dev_handle, buffer, 5) != ESP_OK) {
//         ESP_LOGE(TAG, "Failed to write PDO");
//     }
// }

// static USB_PD_SRC_PDOTypeDef stusb4500_createFixedPDO(uint16_t milli_volts, uint16_t milli_amps)
// {
//     USB_PD_SRC_PDOTypeDef retval = {
//         .d32 = 0,
//     };

//     retval.fix.Voltage = milli_volts / 50;
//     retval.fix.Max_Operating_Current = milli_amps / 10;

//     return retval;
// }

// struct src_pdo_sort
// {
//     uint8_t idx;
//     uint16_t milli_volts;
//     uint16_t milli_amps;
// };

// int pdo_sort_cmp(const void * a, const void * b)
// {
//     int ret;
//     struct src_pdo_sort * pdoa = (struct src_pdo_sort *) a;
//     struct src_pdo_sort * pdob = (struct src_pdo_sort *) b;

//     int a_watts = (int) pdoa->milli_volts * (int) pdoa->milli_amps;
//     int b_watts = (int) pdob->milli_volts * (int) pdob->milli_amps;

//     ret = b_watts - a_watts;
//     if (ret == 0) {
//         // sort by secondary key: lowest voltage
//         ret = pdoa->milli_volts - pdob->milli_volts;
//     }

//     return ret;
// }

// static struct src_pdo_sort sort_pdo()
// {
//     struct src_pdo_sort my_list[10];

//     for (int i = 0; i < sNumSourcePDOsAvailable; i++) {
//         uint16_t milli_volts;
//         uint16_t milli_amps;

//         switch (sSourcePDOs[i].fix.FixedSupply) {
//         case 0:
//             // fixed supply
//             milli_volts = sSourcePDOs[i].fix.Voltage * 50;
//             milli_amps = sSourcePDOs[i].fix.Max_Operating_Current * 10;
//             break;

//         case 1:
//             // Variable supply
//             milli_volts = min(sSourcePDOs[i].var.Min_Voltage, sSourcePDOs[i].var.Max_Voltage) * 50;
//             milli_amps = sSourcePDOs[i].var.Operating_Current * 10;
//             break;

//         default:
//             ESP_LOGW(TAG, "unhandled PDO type: %d", sSourcePDOs[i].fix.FixedSupply);
//             milli_volts = 0;
//             milli_amps = 0;
//             break;
//         }
//         my_list[i].idx = i;
//         my_list[i].milli_volts = milli_volts;
//     }

//     qsort(my_list, sNumSourcePDOsAvailable, sizeof(struct src_pdo_sort), pdo_sort_cmp);

//     return my_list[0];
// }

// static void read_status_registers()
// {
//     uint8_t scratch[40];
//     esp_err_t ret;

//     ESP_GOTO_ON_ERROR(i2c_bitaxe_register_read(stusb4500_dev_handle, REG_ALERT_STATUS_1, scratch, 2), exit, TAG,
//                       "reading status reg");

//     ESP_LOGD(TAG, "ALERT_STATUS: 0x%x", scratch[0]);
//     ESP_LOGD(TAG, "ALERT_STATUS_MASK: 0x%x", scratch[1]);

//     status.d8 = scratch[0] & ~scratch[1];

//     if (status.b.MONITORING_STATUS_AL) {
//         i2c_bitaxe_register_read(stusb4500_dev_handle, REG_MONITORING_STATUS_0, scratch, 2);
//         ESP_LOGD(TAG, "Monitoring Status: 0x%x 0x%x", scratch[0], scratch[1]);
//     }

//     if (status.b.HW_FAULT_STATUS_AL) {
//         i2c_bitaxe_register_read(stusb4500_dev_handle, REG_CC_HW_FAULT_STATUS_0, scratch, 2);
//         ESP_LOGD(TAG, "CC_HW_FAULT Status: 0x%x 0x%x", scratch[0], scratch[1]);
//     }

//     if (status.b.PRT_STATUS_AL) {

//         ESP_GOTO_ON_ERROR(i2c_bitaxe_register_read(stusb4500_dev_handle, REG_PRT_STATUS, scratch, 1), exit, TAG,
//                           "reading PRT_status reg");
//         PRT_status.d8 = scratch[0];

//         ESP_LOGD(TAG, "PRT_STATUS: 0x%x", PRT_status.d8);

//         if (PRT_status.b.MSG_RECEIVED) {
//             USBPD_MsgHeader_TypeDef header;

//             ESP_GOTO_ON_ERROR(i2c_bitaxe_register_read(stusb4500_dev_handle, REG_RX_HEADER, scratch, 2), exit, TAG,
//                               "reading RX_HEADER");
//             header.d16 = LE16(&scratch[0]);

//             ESP_LOGD(TAG, "RX_HEADER: 0x%x num: %d", header.d16, header.b.NumberOfDataObjects);

//             if (header.b.NumberOfDataObjects > 0) {
//                 switch (header.b.MessageType) {
//                 case USBPD_DATAMSG_Source_Capabilities:
//                     ESP_GOTO_ON_ERROR(
//                         i2c_bitaxe_register_read(stusb4500_dev_handle, REG_RX_DATA_OBJ, scratch, header.b.NumberOfDataObjects *
//                         4), exit, TAG, "read RX_DATA_OBJ");

//                     for (int i = 0; i < header.b.NumberOfDataObjects; i++) {
//                         sSourcePDOs[i].d32 = LE32(&scratch[i * 4]);
//                         ESP_LOGD(TAG, "RX_DATA_OBJ[%d]: 0x%lx", i, sSourcePDOs[i].d32);
//                     }
//                     sNumSourcePDOsAvailable = header.b.NumberOfDataObjects;

//                     break;
//                 default:
//                     break;
//                 }
//             }
//         }
//     }

// exit:
// }

// static async service_irq(struct async * pt)
// {
//     async_begin(pt)

//         while (true)
//     {
//         await(alert_count > 0);

//         read_status_registers();
//         alert_count--;
//     }

//     async_end
// }

// static async run(struct async * pt)
// {
//     struct src_pdo_sort pdo;
//     USB_PD_SRC_PDOTypeDef pdo_usbpd;

//     async_begin(pt)

//         stusb4500_softReset();

//     await(sNumSourcePDOsAvailable > 0);

//     pdo = sort_pdo();
//     pdo_usbpd = stusb4500_createFixedPDO(pdo.milli_volts, pdo.milli_amps);
//     stusb4500_writePDO(1, pdo_usbpd);
//     stusb4500_setPDOCount(2);
//     stusb4500_softReset();

//     print_power_contract();

//     async_end
// }

// static void stusb4500_task(void * params)
// {
//     const TickType_t xBlockTime = pdMS_TO_TICKS(500);
//     uint32_t ulNotifiedValue;
//     esp_err_t ret;
//     struct async run_task;

//     async_init(&run_task);

//     for (;;) {

//         run(&run_task);

//         ulNotifiedValue = ulTaskNotifyTake(pdTRUE, xBlockTime);

//         if (ulNotifiedValue & TASK_NOTIFY_ISR_ALART) {
//             read_status_registers();
//         }
//     }
// }

esp_err_t STUSB4500_init()
{

    ESP_LOGI(TAG, "Initializing STUSB4500");
    if (i2c_bitaxe_add_device(STUSB4500_I2CADDR_DEFAULT, &stusb4500_dev_handle, TAG) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device");
        return ESP_FAIL;
    }

    if (!stusb4500_ready()) {
        ESP_LOGE(TAG, "STUSB4500 not found");
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

    _srcCapRequestMax = DEFAULT_SRC_CAP_REQ_MAX;

    USBPDStatus_init(&_status);
    USBPDStateMachine_init(&_state);

    stusb4500_setPDOSnkCount(1);
    stusb4500_updatePDOSnk();
    stusb4500_updateRDOSnk();
    stusb4500_clearAlerts(true);
    stusb4500_updatePrtStatus();
    stusb4500_clearPDOSrc();

    CableStatus_t cable = stusb4500_cableStatus();
    if (CABLE_CONNECTED(cable)) {
        stusb4500_updatePDOSrc();
    }

    // sPower_ready_sem = xSemaphoreCreateBinary();

    // stusb4500_setPDOCount(2);
    // stusb4500_writePDO(0, stusb4500_createFixedPDO(5000, 1500));
    // stusb4500_writePDO(1, stusb4500_createFixedPDO(15000, 1000));
    // stusb4500_softReset();

    // print_power_contract();

    // i2c_bitaxe_register_read(stusb4500_dev_handle, REG_PORT_STATUS_1, scratch, 10);
    // ESP_LOG_BUFFER_HEX_LEVEL(TAG, scratch, 10, ESP_LOG_DEBUG);

    // struct async irq_async;
    // struct async run_async;

    // async_init(&irq_async);
    // async_init(&run_async);

    // while (true) {
    //     service_irq(&irq_async);
    //     run(&run_async);
    //     vTaskDelay(1);
    // }

    // stusb4500_softReset();

    // xTaskCreate(stusb4500_task, TAG, 4096, NULL, 10, &taskHandle);
    // stusb4500_createFixedPDO(5000, 500);
    // stusb4500_setPDOCount(1);

    return ESP_OK;
}

void USBPDStatus_init(USBPDStatus_t * status)
{
    status->hwReset = 0U;
    status->hwFaultStatus.d8 = 0U;
    status->monitoringStatus.d8 = 0U;
    status->ccDetectionStatus.d8 = 0U;
    status->ccStatus.d8 = 0U;
    status->prtStatus.d8 = 0U;
    status->phyStatus.d8 = 0U;
    status->rdoSnk.d32 = 0U;

    status->pdoSnkCount = 0U;
    for (size_t i = 0U; i < NVM_SNK_PDO_MAX; ++i) {
        status->pdoSnk[i].d32 = 0U;
    }

    status->pdoSrcCount = 0U;
    for (size_t i = 0U; i < NVM_SRC_PDO_MAX; ++i) {
        status->pdoSrc[i].d32 = 0U;
    }
}

void USBPDStateMachine_init(USBPDStateMachine_t * state)
{
    state->alertReceived = 0U;
    state->attachReceived = 0U;
    state->irqReceived = 0U;
    state->irqHardReset = 0U;
    state->attachTransition = 0U;
    state->srcPDOReceived = 0U;
    state->srcPDORequesting = 0U;
    state->psrdyReceived = 0U;
    state->msgReceived = 0U;
    state->msgAccept = 0U;
    state->msgReject = 0U;
    state->msgGoodCRC = 0U;

    state->msgHead = 0U;
    state->msgTail = 0U;
    for (size_t i = 0U; i < USBPD_MESSAGE_QUEUE_SZ; ++i) {
        state->msg[i] = 0U;
    }

    state->irqHead = 0U;
    state->irqTail = 0U;
    for (size_t i = 0U; i < USBPD_INTERRUPT_QUEUE_SZ; ++i) {
        state->irq[i] = 0U;
    }
}

void PDO_init(PDO_t * pdo, size_t const number, uint16_t const voltage_mV, uint16_t const current_mA, uint16_t const maxCurrent_mA)
{
    pdo->number = number;
    pdo->voltage_mV = voltage_mV;
    pdo->current_mA = current_mA;
    pdo->maxCurrent_mA = maxCurrent_mA;
}

void PDO_init_zero(PDO_t * pdo)
{
    pdo->number = 0U;
    pdo->voltage_mV = 0U;
    pdo->current_mA = 0U;
    pdo->maxCurrent_mA = 0U;
}
