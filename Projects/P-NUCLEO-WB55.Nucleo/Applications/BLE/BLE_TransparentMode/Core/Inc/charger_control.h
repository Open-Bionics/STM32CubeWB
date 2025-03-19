/*
 * charger_control.h
 *
 *  Created on: Oct 15, 2024
 *      Author: graha
 */

#ifndef INC_CHARGER_CONTROL_H_
#define INC_CHARGER_CONTROL_H_

// nested include headers
#include <stdint.h>
#include <stdbool.h>

extern void CHGC_Init(void);

extern bool CHGC_ChargingIsActive(void);
extern bool CHGC_DeviceIsOnCharger(void);

// Functions used by the CLI
extern bool CHGC_ReadBQ25120ARegister(uint8_t address, uint8_t * read_val_out_ptr);
extern bool CHGC_WriteBQ25120ARegister(uint8_t address, uint8_t new_val);
extern float CHGC_BQ25120GetCalculatedBatteryVoltage(void);

extern void CHGC_UpdateChargeIntStatus(bool low);
extern void CHGC_UpdateWirelessPowerGoodIntStatus(bool low);
extern void CHGC_UpdateChargerPowerGoodIntStatus(bool low);
extern void CHGC_UpdateTemperature(float t);
extern void CHGC_UpdateRawBatteryVoltage(float v);

extern bool CHGC_PowerDownSystem(void);

extern void CHGC_PutChargerIntoErrorSafeState(void);

extern void CHGC_ChargerStatusMonitoringThread(void);

#endif /* INC_CHARGER_CONTROL_H_ */
