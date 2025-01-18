#ifndef STUSB4500_DEF_H_
#define STUSB4500_DEF_H_

#define REG_PORT_STATUS_0 0x0D
#define REG_MONITORING_STATUS_0 0x0F
#define REG_PRT_STATUS 0x16
#define REG_PD_COMMAND_CTRL 0x1A
#define REG_DPM_PDO_NUM 0x70
#define REG_STUSB_GEN1S_RESET_CTRL 0x23
#define REG_DEVICE_ID 0x2f
#define REG_RX_HEADER 0x31
#define REG_RX_DATA_OBJ 0x33
#define REG_TX_HEADER_LOW 0x51
#define REG_TX_HEADER_HIGH 0x52

#define REG_ALERT_STATUS_1 0x0B
#define REG_ALERT_STATUS_MASK 0x0C
typedef union
{
    uint8_t d8;
    struct
    {
        uint8_t PHY_STATUS_AL : 1;
        uint8_t PRT_STATUS_AL : 1;
        uint8_t _Reserved_2 : 1;
        uint8_t PD_TYPEC_STATUS_AL : 1;
        uint8_t HW_FAULT_STATUS_AL : 1;
        uint8_t MONITORING_STATUS_AL : 1;
        uint8_t CC_DETECTION_STATUS_AL : 1;
        uint8_t HARD_RESET_AL : 1;
    } b;
} STUSB_GEN1S_ALERT_STATUS_RegTypeDef;

#define REG_PORT_STATUS_1 0x0E
typedef union
{
    uint8_t d8;
    struct
    {
        uint8_t CC_ATTACH_STATE : 1;
        uint8_t CC_VCONN_SUPPLY_STATE : 1;
        uint8_t CC_DATA_ROLE : 1;
        uint8_t CC_POWER_ROLE : 1;
        uint8_t START_UP_POWER_MODE : 1;
        uint8_t CC_ATTACH_MODE : 3;
    } b;
} STUSB_GEN1S_CC_DETECTION_STATUS_RegTypeDef;

typedef union
{
    uint8_t d8;
    struct
    {
        uint8_t HWRESET_RECEIVED : 1;
        uint8_t HWRESET_DONE : 1;
        uint8_t MSG_RECEIVED : 1;
        uint8_t MSG_SENT : 1;
        uint8_t BIST_RECEIVED : 1;
        uint8_t BIST_SENT : 1;
        uint8_t Reserved_6 : 1;
        uint8_t TX_ERROR : 1;
    } b;
} STUSB_GEN1S_PRT_STATUS_RegTypeDef;

typedef union
{
    uint8_t d8;
    struct
    {
        uint8_t PHY_STATUS_AL_MASK : 1;
        uint8_t PRT_STATUS_AL_MASK : 1;
        uint8_t _Reserved_2 : 1;
        uint8_t PD_TYPEC_STATUS_AL_MASK : 1;
        uint8_t HW_FAULT_STATUS_AL_MASK : 1;
        uint8_t MONITORING_STATUS_AL_MASK : 1;
        uint8_t CC_DETECTION_STATUS_AL_MASK : 1;
        uint8_t HARD_RESET_AL_MASK : 1;
    } b;
} STUSB_GEN1S_ALERT_STATUS_MASK_RegTypeDef;

typedef union
{
    uint32_t d32;
    struct
    {
        uint32_t Max_Operating_Current : 10;
        uint32_t Voltage : 10;
        uint8_t PeakCurrent : 2;
        uint8_t Reserved : 2;
        uint8_t Unchuncked_Extended : 1;
        uint8_t Dual_RoleData : 1;
        uint8_t Communication : 1;
        uint8_t UnconstraintPower : 1;
        uint8_t SuspendSupported : 1;
        uint8_t DualRolePower : 1;
        uint8_t FixedSupply : 2;
    } fix;
    struct
    {
        uint32_t Operating_Current : 10;
        uint32_t Min_Voltage : 10;
        uint32_t Max_Voltage : 10;
        uint8_t VariableSupply : 2;
    } var;
    struct
    {
        uint32_t Operating_Power : 10;
        uint32_t Min_Voltage : 10;
        uint32_t Max_Voltage : 10;
        uint8_t Battery : 2;
    } bat;
    struct
    {
        /*
        uint8_t Max_Current :7;
        uint8_t  Reserved0 :1;
        uint8_t Min_Voltage:8;
        uint8_t  Reserved1 :1;
        uint8_t Max_Voltage:9;
        uint8_t  Reserved2 :2;
        uint8_t ProgDev : 2 ;
        uint8_t Battery:2;
        */
        uint8_t Max_Current : 7;
        uint8_t Reserved0 : 1;
        uint16_t Min_Voltage : 8; /* to prevent packing issue ?? */
        uint8_t Reserved1 : 1;
        uint16_t Max_Voltage : 8;
        uint8_t Reserved2 : 3;
        uint8_t ProgDev : 2;
        uint8_t Battery : 2;

    } apdo;
} USB_PD_SRC_PDOTypeDef;

typedef union
{
    uint16_t d16;
    struct
    {
#if defined(USBPD_REV30_SUPPORT)
        uint16_t MessageType : /*!< Message Header's message Type                      */
                               5;
#else                           /* USBPD_REV30_SUPPORT */
        uint16_t MessageType : /*!< Message Header's message Type                      */
                               4;
        uint16_t Reserved4 : /*!< Reserved                                           */
                             1;
#endif                          /* USBPD_REV30_SUPPORT */
        uint16_t PortDataRole : /*!< Message Header's Port Data Role                    */
                                1;
        uint16_t SpecificationRevision : /*!< Message Header's Spec Revision                     */
                                         2;
        uint16_t PortPowerRole_CablePlug : /*!< Message Header's Port Power Role/Cable Plug field  */
                                           1;
        uint16_t MessageID : /*!< Message Header's message ID                        */
                             3;
        uint16_t NumberOfDataObjects : /*!< Message Header's Number of data object             */
                                       3;
        uint16_t Extended : /*!< Reserved                                           */
                            1;
    } b;
} USBPD_MsgHeader_TypeDef;

#define REG_RDO_STATUS 0x91
typedef union
{
    uint32_t d32;
    struct
    {
        uint32_t MaxCurrent : 10; // Bits 9..0
        uint32_t OperatingCurrent : 10;
        uint8_t reserved_22_20 : 3;
        uint8_t UnchunkedMess_sup : 1;
        uint8_t UsbSuspend : 1;
        uint8_t UsbComCap : 1;
        uint8_t CapaMismatch : 1;
        uint8_t GiveBack : 1;
        uint8_t Object_Pos : 3;  // Bits 30..28 (3-bit)
        uint8_t reserved_31 : 1; // Bits 31

    } b;
} STUSB_GEN1S_RDO_REG_STATUS_RegTypeDef;

#endif // STUSB4500_DEF_H_