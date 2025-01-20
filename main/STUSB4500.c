#include <stdbool.h>
#include <stdlib.h>

#include <driver/gpio.h>
#include <esp_check.h>
#include <esp_log.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <hal/gpio_ll.h>

#include <async.h>

#include "STUSB4500_register.h"
#include "STUSB4500.h"
#include "STUSB4500_def.h"


#include "i2c_bitaxe.h"

#define max(a, b) ((a) > (b) ? (a) : (b))
#define min(a, b) ((a) < (b) ? (a) : (b))

#define STUSB4500_I2CADDR_DEFAULT 0x28
#define ALERT_PIN 14

#define EVAL_DEVICE_ID 0x21 // in device ID reg (0x2F)
#define PROD_DEVICE_ID 0x25 //

#define ESP_INTR_FLAG_DEFAULT 0


#define CABLE_CONNECTED(c) ((CC1Connected == (c)) || (CC2Connected == (c)))

#define TASK_NOTIFY_ISR_ALART (1 << 12)

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

static void read_status_registers();

void stusb4500_setPDOSnkCount(uint8_t count)
{
    ESP_LOGD(TAG, "Setting PDO count to %d", count);
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
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static inline uint16_t bytes_to_le16(uint8_t * bytes)
{
    return (uint16_t) bytes[0] | ((uint16_t) bytes[1] << 8);
}

static inline void le16_to_bytes(uint16_t value, uint8_t * bytes)
{
    bytes[0] = value & 0xff;
    bytes[1] = (value >> 8) & 0xff;
}

static inline uint32_t bytes_to_le32(uint8_t * bytes)
{
    return (uint32_t) bytes[0] | ((uint32_t) bytes[1] << 8) | ((uint32_t) bytes[2] << 16) | ((uint32_t) bytes[3] << 24);
}

static inline void le32_to_bytes(uint32_t value, uint8_t * bytes)
{
    bytes[0] = value & 0xff;
    bytes[1] = (value >> 8) & 0xff;
    bytes[2] = (value >> 16) & 0xff;
    bytes[3] = (value >> 24) & 0xff;
}

static void print_pdo(USB_PD_SNK_PDO_TypeDef pdo){
    switch (pdo.fix.FixedSupply) {
        case PDO_SNK_FIX_FIXED: // fixed supply
            ESP_LOGI(TAG, "Fixed Supply PDO: %d mV %d mA", pdo.fix.Voltage * 50, pdo.fix.Operational_Current * 10);
            break;
        
        case PDO_SNK_FIX_VARIABLE: // Variable Supply
            ESP_LOGI(TAG, "Variable Supply PDO: (%d-%d mV) %d mA", pdo.var.Min_Voltage * 50, pdo.var.Max_Voltage * 50, pdo.var.Operating_Current * 10);
            break;

        case PDO_SNK_FIX_BATTERY: // Battery Supply
            ESP_LOGI(TAG, "Battery Supply PDO: (%d-%d mV) %d mW", pdo.bat.Min_Voltage * 50, pdo.bat.Max_Voltage * 50, pdo.bat.Operating_Power * 250);
            break;

        default:
            ESP_LOGE(TAG, "Unknown PDO type %d", pdo.fix.FixedSupply);
            break;
    }
}

void stusb4500_updatePDOSnk(void)
{
#define BUFF_SZ NVM_SNK_PDO_MAX * sizeof(USB_PD_SNK_PDO_TypeDef)
    uint8_t pdo[BUFF_SZ];
    uint8_t pdoCount;

    stusb4500_waitUntilReady();

    i2c_bitaxe_register_read(stusb4500_dev_handle, DPM_PDO_NUMB, &pdoCount, 1);
    pdoCount &= 0x03;

    i2c_bitaxe_register_read(stusb4500_dev_handle, DPM_SNK_PDO1, pdo, BUFF_SZ);

    ESP_LOGD(TAG, "PDO count: %d", pdoCount);
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, pdo, BUFF_SZ, ESP_LOG_DEBUG);

    _status.pdoSnkCount = pdoCount;
    for (uint8_t i = 0, j = 0; i < NVM_SNK_PDO_MAX; i++, j += 4) {
        if (i < pdoCount) {
            _status.pdoSnk[i].d32 = bytes_to_le32(&pdo[j]);
            print_pdo(_status.pdoSnk[i]);
        } else {
            _status.pdoSnk[i].d32 = 0U;
        }
    }
#undef BUFF_SZ
}

void stusb4500_setPDOSnk(PDO_t pdo)
{
    uint8_t buf[5];

    if ((pdo.number < 1) || (pdo.number > NVM_SNK_PDO_MAX)) {
        ESP_LOGE(TAG, "Invalid PDO number %d", pdo.number);
        return;
    }

    uint16_t voltage_mV = pdo.voltage_mV;
    //  if (1U == pdo.number) // PDO 1 must always be USB +5V
    //    { voltage_mV = 5000U; }

    // use the received PDO definition #1 (USB default) as template for new PDO,
    // just update the voltage and current to the input PDO.
    USB_PD_SNK_PDO_TypeDef def = _status.pdoSnk[0];
    uint8_t address = DPM_SNK_PDO1 + (4U * (pdo.number - 1U));
    def.fix.Voltage = voltage_mV / 50U;
    def.fix.Operational_Current = pdo.current_mA / 10U;

    buf[0] = address;
    le32_to_bytes(def.d32, &buf[1]);

    i2c_bitaxe_register_write_bytes(stusb4500_dev_handle, buf, 5U);
}

void stusb4500_updatePDOSrc()
{
    stusb4500_sendPDCableReset();
    stusb4500_waitUntilReady();
    stusb4500_clearAlerts(true);

    // for(int i=0;i<500;i++) {
    //     read_status_registers();
    // }

    // for(int i=0;i<3;i++) {
    //     stusb4500_sendPDCableReset();
    //     for(int j=0;j<10;j++) {
    //         // xTaskNotifyGive(taskHandle);
    //         read_status_registers();
    //     }
    // }

    

    // DELAY(TLOAD_REG_INIT_MS);

    // uint8_t maxRequests = _srcCapRequestMax;
    // uint8_t request = 0U;

    // ++(_state.srcPDORequesting);

    // while ((0U != _state.srcPDORequesting) && (request < maxRequests)) {
    //     if (!stusb4500_sendPDCableReset()) {
    //         break;
    //     }
    //     processAlerts();
    //     ++request;
    // }
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

        ESP_LOGD(TAG, "RDO: %d %d mV %d mA %d mA", _snkRDO.number, _snkRDO.voltage_mV, _snkRDO.current_mA, _snkRDO.maxCurrent_mA);
    } else {
        PDO_init_zero(&_snkRDO);
        ESP_LOGD(TAG, "no RDO");
    }
}

static void stusb4500_set_irq_mask()
{
    STUSB_GEN1S_ALERT_STATUS_MASK_RegTypeDef alertMask;
    // set interrupts to unmask
    alertMask.d8 = 0xFF;

    // alertMask.b.PHY_STATUS_AL_MASK          = 0U;
    alertMask.b.PRT_STATUS_AL_MASK = 0U;
    // alertMask.b.PD_TYPEC_STATUS_AL_MASK = 0U;
    // alertMask.b.HW_FAULT_STATUS_AL_MASK     = 0U;
    // alertMask.b.MONITORING_STATUS_AL_MASK = 0U;
    alertMask.b.CC_DETECTION_STATUS_AL_MASK = 0U;
    alertMask.b.HARD_RESET_AL_MASK = 0U;

    // unmask the above alarms
    i2c_bitaxe_register_write_byte(stusb4500_dev_handle, ALERT_STATUS_MASK, alertMask.d8);
}

/****
 * clear all pending alerts by reading the status registers.
 ****/
void stusb4500_clearAlerts(bool unmask)
{
    // clear alert status
    uint8_t alertStatus[12];
    i2c_bitaxe_register_read(stusb4500_dev_handle, ALERT_STATUS_1, alertStatus, 12);

    if (unmask) {
        stusb4500_set_irq_mask();
    }
}

void stusb4500_updatePrtStatus(void)
{
    uint8_t prtStatus[10];
    i2c_bitaxe_register_read(stusb4500_dev_handle, PORT_STATUS_TRANS, prtStatus, 10U);

    _status.ccDetectionStatus.d8 = prtStatus[1];
    _status.monitoringStatus.d8 = prtStatus[3];
    _status.ccStatus.d8 = prtStatus[4];
    _status.hwFaultStatus.d8 = prtStatus[6];

    ESP_LOGD(TAG, "CC Detection: 0x%02x", _status.ccDetectionStatus.d8);

    if (_status.ccDetectionStatus.b.CC_ATTACH_STATE == 1) {
        const char* connection_type;
        switch (_status.ccDetectionStatus.b.CC_ATTACH_MODE) {
            case 0:
                connection_type = "None";
                break;
            case 1:
                connection_type = "USB-PD Source";
                break;
            case 3:
                connection_type = "Debug Accessory";
                break;
            default:
                connection_type = "Unknown";
                break;
        }

        ESP_LOGI(TAG, "Cable connected: %s", connection_type);
        ESP_LOGI(TAG, "Power role: %s", _status.ccDetectionStatus.b.CC_POWER_ROLE ? "Sink" : "Source");
        ESP_LOGI(TAG, "Data role: %s", _status.ccDetectionStatus.b.CC_DATA_ROLE ? "DFP" : "UFP");
    } else {
        ESP_LOGI(TAG, "Cable disconnected");
    }

    ESP_LOGD(TAG, "Monitoring: 0x%02x", _status.monitoringStatus.d8);
    ESP_LOGI(TAG, "VBUS: %s", _status.monitoringStatus.b.VBUS_READY ? "Ready" : "Unpowered");
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

static void IRAM_ATTR alert_isr_handler(void * arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    GPIO.status_w1tc = BIT(ALERT_PIN);

    // xTaskNotifyFromISR(taskHandle, TASK_NOTIFY_ISR_ALART, eSetBits, &xHigherPriorityTaskWoken);
    vTaskNotifyGiveFromISR(taskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void STUSB4500_wait_for_power_ready()
{
    while (is_power_ready == 0) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void stusb4500_sw_reset()
{
    uint8_t scratch[12];
    int i;

    ESP_LOGD(TAG, "resetting STUSB4500");

    i2c_bitaxe_register_write_byte(stusb4500_dev_handle, STUSB_GEN1S_RESET_CTRL_REG, 1);

    vTaskDelay(pdMS_TO_TICKS(27));

    while(i2c_bitaxe_register_read(stusb4500_dev_handle, REG_DEVICE_ID, scratch, 1) != ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGD(TAG, "back from reset");

    // for (i = 0; i < 12; i++) {
    //     i2c_bitaxe_register_read(stusb4500_dev_handle, ALERT_STATUS_1 + 1, scratch, 1);
    // }

    vTaskDelay(pdMS_TO_TICKS(27));

    i2c_bitaxe_register_write_byte(stusb4500_dev_handle, STUSB_GEN1S_RESET_CTRL_REG, 0);
}

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
    ESP_LOGD(TAG, "sending PD cable reset");
    CableStatus_t cable = stusb4500_cableStatus();
    if (!CABLE_CONNECTED(cable)) {
        ESP_LOGE(TAG, "Cable not connected");
        return false;
    }

    // send PD message "soft reset" to source by setting TX header (0x51) to 0x0D,
    // and set PD command (0x1A) to 0x26.

    if (i2c_bitaxe_register_write_byte(stusb4500_dev_handle, TX_HEADER, 0x0D) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write TX_HEADER");
        return false;
    }

    if (i2c_bitaxe_register_write_byte(stusb4500_dev_handle, TX_HEADER+1, 0x00) != ESP_OK) {
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

static void read_status_registers()
{
    STUSB_GEN1S_ALERT_STATUS_RegTypeDef alertStatus;
    STUSB_GEN1S_ALERT_STATUS_MASK_RegTypeDef alertMask;
    uint8_t scratch[40];
    esp_err_t ret;

    i2c_bitaxe_register_read(stusb4500_dev_handle, ALERT_STATUS_1, scratch, 2);

    alertMask.d8 = scratch[1];
    // alertStatus.d8 = scratch[0] & ~(alertMask.d8);
    alertStatus.d8 = scratch[0];

    ESP_LOGD(TAG, "ALERT_STATUS: 0x%x 0x%x", scratch[0], scratch[1]);

    // if (scratch[1] & (1 << 1)) {
    //     stusb4500_set_irq_mask();
    // }

    if (alertStatus.b.PRT_STATUS_AL) {

        ESP_LOGD(TAG, "PRT_STATUS_AL");

        ESP_GOTO_ON_ERROR(i2c_bitaxe_register_read(stusb4500_dev_handle, PRT_STATUS, scratch, 1), exit, TAG,
                          "reading PRT_status reg");
        _status.prtStatus.d8 = scratch[0];

        ESP_LOGD(TAG, "PRT_STATUS: 0x%x", _status.prtStatus.d8);

        if (_status.prtStatus.b.MSG_RECEIVED) {
            ESP_LOGD(TAG, "MSG_RECEIVED");
            USBPDMessageHeader_t header;

            ESP_GOTO_ON_ERROR(i2c_bitaxe_register_read(stusb4500_dev_handle, RX_HEADER, scratch, 2), exit, TAG,
                              "reading RX_HEADER");
            header.d16 = bytes_to_le16(scratch);

            ESP_LOGD(TAG, "RX_HEADER: 0x%x num: %d", header.d16, header.b.dataObjectCount);

            if (header.b.dataObjectCount > 0) {

                ESP_GOTO_ON_ERROR(i2c_bitaxe_register_read(stusb4500_dev_handle, RX_BYTE_CNT, scratch, 1), exit, TAG,
                                  "read RX_BYTE_CNT");
                uint8_t byte_count = scratch[0];
                if (byte_count != header.b.dataObjectCount * 4) {
                    ESP_LOGE(TAG, "byte count mismatch: %d != %d", byte_count, header.b.dataObjectCount * 4);
                    goto exit;
                }

                switch (header.b.messageType) {
                    case USBPD_DATAMSG_Source_Capabilities:
                        ESP_GOTO_ON_ERROR(i2c_bitaxe_register_read(stusb4500_dev_handle, RX_DATA_OBJ, scratch, byte_count), exit, TAG,
                                        "read RX_DATA_OBJ");

                        _status.pdoSrcCount = header.b.dataObjectCount;
                        for (uint8_t i = 0U, j = 0U; i < header.b.dataObjectCount; ++i, j += 4) {
                            _status.pdoSrc[i].d32 = bytes_to_le32(&scratch[j]);
                            if (0U == i) {
                                _status.pdoSrc[i].fix.Voltage = 100U;
                            }
                            PDO_init(&_srcPDO[i], i + 1, _status.pdoSrc[i].fix.Voltage * 50U,
                                    _status.pdoSrc[i].fix.Max_Operating_Current * 10U, 0);
                        }

                        _state.srcPDORequesting = 0U;
                        ++(_state.srcPDOReceived);
                        break;
                    default:
                        break;
                }
            }
        }
    }


    if (alertStatus.b.CC_DETECTION_STATUS_AL) {
        i2c_bitaxe_register_read(stusb4500_dev_handle, PORT_STATUS_TRANS, scratch, 2);
        _status.ccDetectionStatus.d8 = scratch[1];

        if (scratch[0] & STUSBMASK_ATTACH_STATUS_TRANS) {
            ESP_LOGD(TAG, "CC attachement transaction");
            stusb4500_set_irq_mask();
        }
    }

    if (alertStatus.b.MONITORING_STATUS_AL) {
        // ESP_LOGD(TAG, "read Monitoring Status");
        i2c_bitaxe_register_read(stusb4500_dev_handle, TYPEC_MONITORING_STATUS_0, scratch, 2);
        // ESP_LOGD(TAG, "Monitoring Status: 0x%x 0x%x", scratch[0], scratch[1]);
        _status.monitoringStatus.d8 = scratch[1];
    }

    // always read & update CC attachement status
    i2c_bitaxe_register_read(stusb4500_dev_handle, CC_STATUS, scratch, 1);
    _status.ccStatus.d8 = scratch[0];

    if (alertStatus.b.HW_FAULT_STATUS_AL) {
        // ESP_LOGD(TAG, "read HW Fault Status");
        i2c_bitaxe_register_read(stusb4500_dev_handle, CC_HW_FAULT_STATUS_0, scratch, 2);
        // ESP_LOGD(TAG, "CC_HW_FAULT Status: 0x%x 0x%x", scratch[0], scratch[1]);
        _status.hwFaultStatus.d8 = scratch[1];
    }

exit:
}

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

static void stusb4500_task(void * params)
{
    const TickType_t xBlockTime = pdMS_TO_TICKS(3000);
    uint32_t ulNotifiedValue;
    esp_err_t ret;

    int num_resets = 0;

    for (;;) {

        ulNotifiedValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // ESP_LOGI(TAG, "Alert received: %ld", ulNotifiedValue);
        while(ulNotifiedValue) {
            ulNotifiedValue--;
            read_status_registers();
        }
        

        // if (ulNotifiedValue & TASK_NOTIFY_ISR_ALART) {
            
        // }

        if (num_resets == 0) {
            num_resets++;
            // vTaskDelay(pdMS_TO_TICKS(100));
            // stusb4500_sw_reset();

            
            // stusb4500_sendPDCableReset();
        }
        
    }
}

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

    xTaskCreate(stusb4500_task, TAG, 4096, NULL, 24, &taskHandle);

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ALERT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
        // .intr_type = GPIO_INTR_LOW_LEVEL,
    };
    gpio_config(&io_conf);

    gpio_isr_register(alert_isr_handler, NULL, ESP_INTR_FLAG_DEFAULT, NULL);

    // ESP_RETURN_ON_ERROR(gpio_isr_handler_add(ALERT_PIN, alert_isr_handler, NULL), TAG, "adding ISR handler");

    _srcCapRequestMax = DEFAULT_SRC_CAP_REQ_MAX;

    USBPDStatus_init(&_status);
    USBPDStateMachine_init(&_state);

    stusb4500_updatePDOSnk();
    stusb4500_updateRDOSnk();
    stusb4500_clearAlerts(true);
    stusb4500_updatePrtStatus();
    stusb4500_clearPDOSrc();

    CableStatus_t cable = stusb4500_cableStatus();
    if (CABLE_CONNECTED(cable)) {
        ESP_LOGI(TAG, "Cable connected");
        stusb4500_setPDOSnkCount(1);
        stusb4500_updatePDOSrc();
    }

    vTaskDelay(pdMS_TO_TICKS(2000));

    stusb4500_updatePDOSnk();
    stusb4500_updateRDOSnk();
    stusb4500_updatePrtStatus();

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

    // stusb4500_createFixedPDO(5000, 500);
    // stusb4500_setPDOCount(1);

    STUSB4500_wait_for_power_ready();

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
