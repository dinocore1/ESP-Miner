#ifndef STUSB4500_DEF_H_
#define STUSB4500_DEF_H_

#define USBPD_REV30_SUPPORT 1
#define NVM_SNK_PDO_MAX 3
#define NVM_SRC_PDO_MAX 10

typedef struct USBPDStatus
{
    uint8_t hwReset;
    STUSB_GEN1S_HW_FAULT_STATUS_RegTypeDef hwFaultStatus;         // 8-bit
    STUSB_GEN1S_MONITORING_STATUS_RegTypeDef monitoringStatus;    // 8-bit
    STUSB_GEN1S_CC_DETECTION_STATUS_RegTypeDef ccDetectionStatus; // 8-bit
    STUSB_GEN1S_CC_STATUS_RegTypeDef ccStatus;                    // 8-bit
    STUSB_GEN1S_PRT_STATUS_RegTypeDef prtStatus;                  // 8-bit
    STUSB_GEN1S_PHY_STATUS_RegTypeDef phyStatus;                  // 8-bit
    STUSB_GEN1S_RDO_REG_STATUS_RegTypeDef rdoSnk;
    size_t pdoSnkCount;
    USB_PD_SNK_PDO_TypeDef pdoSnk[NVM_SNK_PDO_MAX];
    size_t pdoSrcCount;
    USB_PD_SRC_PDO_TypeDef pdoSrc[NVM_SRC_PDO_MAX];

} USBPDStatus_t;

void USBPDStatus_init(USBPDStatus_t * status);

typedef struct PDO
{
    size_t number;
    uint16_t voltage_mV;
    uint16_t current_mA;
    uint16_t maxCurrent_mA;
} PDO_t;

void PDO_init(PDO_t * pdo, size_t const number, uint16_t const voltage_mV, uint16_t const current_mA, uint16_t const maxCurrent_mA);

void PDO_init_zero(PDO_t * pdo);

void stusb4500_setPDOSnkCount(uint8_t const count);

/**
 * query the internal device ID register of the STUSB4500 and verify it matches
 * the expected manufacturer-specified ID. this is used to determine if the
 * device has powered on and can respond to I2C read requests.
 */
bool stusb4500_ready(void);

void stusb4500_waitUntilReady(void);

void stusb4500_updatePDOSnk();

void stusb4500_updateRDOSnk();

/**
 * clear all pending alerts by reading the status registers.
 */
void stusb4500_clearAlerts(bool const unmask);

#endif // STUSB4500_DEF_H_
