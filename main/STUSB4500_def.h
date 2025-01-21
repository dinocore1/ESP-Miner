#ifndef STUSB4500_DEF_H_
#define STUSB4500_DEF_H_

#define USBPD_REV_3_0_SUPPORT 0
#define USBPD_MESSAGE_QUEUE_SZ 32
#define USBPD_INTERRUPT_QUEUE_SZ 32
#define NVM_SNK_PDO_MAX 3
#define NVM_SRC_PDO_MAX 10
#define DEFAULT_SRC_CAP_REQ_MAX 200

typedef union
{
    uint16_t d16;
    struct
    {
#if defined(USBPD_REV_3_0_SUPPORT)
        uint16_t messageType : 5; // USBPD rev >= 3.0 message type
#else
        uint16_t messageType : 4; // USBPD rev  < 3.0 message type
        uint16_t reserved_4 : 1;  // reserved
#endif
        uint16_t portDataRole : 1;    // port data role
        uint16_t specRevision : 2;    // spec revision
        uint16_t portPowerRole : 1;   // port power role/cable plug
        uint16_t messageID : 3;       // message ID
        uint16_t dataObjectCount : 3; // number of data objects
        uint16_t extended : 1;        // reserved
    } b;
} USBPDMessageHeader_t;

_Static_assert(sizeof(USBPDMessageHeader_t) == 2, "sizeof(USBPDMessageHeader_t) != 2");

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

typedef struct USBPDStateMachine
{
    volatile uint8_t alertReceived;
    volatile uint8_t attachReceived;
    uint16_t irqReceived;
    uint16_t irqHardReset;
    uint16_t attachTransition;
    uint16_t srcPDOReceived;
    uint16_t srcPDORequesting;
    uint16_t psrdyReceived;
    uint16_t msgReceived;
    uint16_t msgAccept;
    uint16_t msgReject;
    uint16_t msgGoodCRC;
    uint8_t msgHead;
    uint8_t msgTail;
    uint8_t msg[USBPD_MESSAGE_QUEUE_SZ];
    uint8_t irqHead;
    uint8_t irqTail;
    uint8_t irq[USBPD_INTERRUPT_QUEUE_SZ];

} USBPDStateMachine_t;

void USBPDStateMachine_init(USBPDStateMachine_t *);

typedef struct PDO
{
    size_t number;
    uint16_t voltage_mV;
    uint16_t current_mA;
    uint16_t maxCurrent_mA;
} PDO_t;

void PDO_init(PDO_t * pdo, size_t const number, uint16_t const voltage_mV, uint16_t const current_mA, uint16_t const maxCurrent_mA);

void PDO_init_zero(PDO_t * pdo);

typedef enum CableStatus
{
    NONE = -1,
    NotConnected, // = 0
    CC1Connected, // = 1
    CC2Connected, // = 2
    COUNT         // = 3
} CableStatus_t;

void stusb4500_setPDOSnkCount(uint8_t const count);

/**
 * query the internal device ID register of the STUSB4500 and verify it matches
 * the expected manufacturer-specified ID. this is used to determine if the
 * device has powered on and can respond to I2C read requests.
 */
bool stusb4500_ready(void);

void stusb4500_waitUntilReady(void);

void stusb4500_updatePDOSnk();

void stusb4500_updatePDOSrc(void);

void stusb4500_updateRDOSnk();

void stusb4500_setPDOSnk(PDO_t pdo);

/**
 * clear all pending alerts by reading the status registers.
 */
void stusb4500_clearAlerts(bool unmask);

void stusb4500_updatePrtStatus(void);

void stusb4500_clearPDOSrc(void);

/**
 * read the port and Type-C status registers to determine if a USB cable is
 * connected to the STUSB4500 device. if connected, the orientation is
 * determined and indicated by return value of the selected CC line.
 */
CableStatus_t stusb4500_cableStatus(void);

bool stusb4500_sendPDCableReset();

#endif // STUSB4500_DEF_H_
