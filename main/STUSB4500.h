#ifndef STUSB4500_H_
#define STUSB4500_H_

#include <esp_err.h>

esp_err_t STUSB4500_init(void);

void STUSB4500_wait_for_power_ready();

#endif // STUSB4500_H_