#ifndef STUSB4500_DEF_H_
#define STUSB4500_DEF_H_

#define REG_DPM_PDO_NUM 0x70


typedef union
{
  uint32_t d32;
  struct
  {
    //Table 6-9 Fixed Supply PDO - Source
    uint32_t Max_Operating_Current : 10; // bits 9..0
    uint32_t Voltage               : 10; // bits 19..10
    uint8_t  PeakCurrent           :  2; // bits 21..20
    uint8_t  Reserved              :  3; //
    uint8_t  DataRoleSwap          :  1; // bits 25
    uint8_t  Communication         :  1; // bits 26
    uint8_t  ExternalyPowered      :  1; // bits 27
    uint8_t  SuspendSuported       :  1; // bits 28
    uint8_t  DualRolePower         :  1; // bits 29
    uint8_t  FixedSupply           :  2; // bits 31..30
  } fix;
  struct
  {
    //Table 6-11 Variable Supply (non-Battery) PDO - Source
    uint32_t Operating_Current : 10;
    uint32_t Min_Voltage       : 10;
    uint32_t Max_Voltage       : 10;
    uint8_t  VariableSupply    :  2;
  } var;
  struct
  {
    //Table 6-12 Battery Supply PDO - Source
    uint32_t Operating_Power : 10;
    uint32_t Min_Voltage     : 10;
    uint32_t Max_Voltage     : 10;
    uint8_t  Battery         :  2;
  } bat;

} USB_PD_SRC_PDO_TypeDef;

#endif // STUSB4500_DEF_H_