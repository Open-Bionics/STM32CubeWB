/*
 * charger_control.c
 *
 *  Created on: Oct 15, 2024
 *      Author: graha
 */

// Included header files
#include "charger_control.h"
#include "BQ25120A_Battery_Charger.h"
#include "main.h"
#include "led.h"

// #define CHGC_SET_DEBUG_OUTPUT

#ifndef CHGC_SET_DEBUG_OUTPUT
#define CHARGER_GPIO_DBG(...)
#define CHARGER_COMMS_DBG(...)
#define CHARGER_ERROR_DBG(...)
#define CHARGER_STATE_MACHINE_DBG(...)
#else
#define CHARGER_GPIO_DBG                       printf
#define CHARGER_COMMS_DBG                      printf
#define CHARGER_ERROR_DBG                      printf
#define CHARGER_STATE_MACHINE_DBG              printf
#endif

#define CHGC_MAX_CHARGER_REQUESTS		10

#define CHGC_CHANGE_IMPACTING_STATUS_MONITOR_FLAG		1u

#define CHGC_MAXIMUM_CHARGE_TIME_MS		(6 * 60 * 60 * 1000)		// 6 hours

typedef bool (*chgc_charger_coms_fptr)(void);

typedef enum _CHG_MODE
{
	CHGC_CHARGER_POWERED_FROM_WIRELESS_POWER = 0,
	CHGC_CHARGING_ENABLED = CHGC_CHARGER_POWERED_FROM_WIRELESS_POWER,
	CHGC_CHARGER_POWERED_FROM_BATTERY = 1,
	CHGC_CHARGING_DISABLED = CHGC_CHARGER_POWERED_FROM_BATTERY,
}chgc_charger_mode_t;

typedef enum _CHARGER_STATUS_CHANGED
{
	CHGC_STATUS_GPIO_CHANGED_FLAG = 1u,
	CHGC_STATUS_REQUESTS_PENDING_FLAG = 2u,
}chgc_status_changed_flags_t;

// TODO - this charger state machine needs revisiting to cater for edge cases
typedef enum _CHARGER_STATUS_STATE_MACHINE
{
	CHGC_STATE_NOT_ON_CHARGER,
	CHGC_STATE_READY_TO_CHARGE,
	CHGC_STATE_CHARGING_ACTIVE_PHASE,
	CHGC_STATE_CHARGING_INACTIVE_PHASE,
	CHGC_STATE_ERROR,
}chgc_state_t;

// Private function declarations

static void DisableWirelessPower(void);
static void EnableWirelessPower(void);
static void SetChargerPoweredFromBattery(void);
static void SetChargerPoweredFromWirelessPower(void);

void ChargerCommunicationsThread(void *argument);
void ChargerStatusMonitoringThread(void *argument);

static void fault_cb(BQ25120A_fault_t fault);

static bool DefaultChargerRequestFunction(void);

static bool AddChargerRequestToQueue(chgc_charger_coms_fptr fptr);
static bool AddChargerRequestToQueueUnsafe(chgc_charger_coms_fptr fptr);

static bool PowerDownRequest(void);

static BQ25120A_params_t bq_params;

static const float _TARGET_CHARGE_VOLTAGE = 4.3f;
static const float _TARGET_CHARGE_CURRENT_MA = 40.0f;
static const float _TARGET_PRECHARGE_CURRENT_MA = 4.0f;

// charger error conditions
static const float _TOO_HIGH_CHARGE_VOLTAGE = 4.4f;
static const float _TOO_HIGH_CHARGE_VOLTAGE_HYSTERISIS = 0.1f;
static const float _TOO_HOT_TEMPERATURE = 52.0f;
static const float _TOO_HOT_TEMPERATURE_HYSTERISIS = 5.0f;

static chgc_charger_mode_t _charger_mode = CHGC_CHARGER_POWERED_FROM_WIRELESS_POWER;

static chgc_charger_coms_fptr _charger_requests[CHGC_MAX_CHARGER_REQUESTS] = {0};
static uint8_t _charger_request_ip = 0;
static uint8_t _charger_request_op = 0;
static uint8_t _charger_request_count = 0;

static float _latest_read_battery_voltage = 0.0f;

// flags that control charger state
static bool _charging_power_active = false;
static bool _wireless_power_active = false;
static bool _bq25120a_temperature_too_high = false;
static bool _bq25120a_fault_detected = false;
static bool _secondary_temperature_too_high = false;
static bool _battery_voltage_too_high = false;

// summarised charger state
static bool _charging_active = false;

static chgc_state_t _charger_state = CHGC_STATE_NOT_ON_CHARGER;

// Public Functions

/**
 * @brief Module Initialisation function
 */
void CHGC_Init(void)
{
	_charger_mode = CHGC_CHARGER_POWERED_FROM_WIRELESS_POWER;
	_latest_read_battery_voltage = 0.0f;
	_charging_power_active = false;
	_wireless_power_active = false;
	_bq25120a_temperature_too_high = false;
	_bq25120a_fault_detected = false;
	_secondary_temperature_too_high = false;
	_battery_voltage_too_high = false;

	SetChargerPoweredFromBattery();

	BQ25120A_init(&bq_params, MCU_CHG_INT_GPIO_Port, MCU_CHG_INT_Pin, MCU_CHG_nPGD_GPIO_Port, MCU_CHG_nPGD_Pin, MCU_CHG_nDISABLE_GPIO_Port, MCU_CHG_nDISABLE_Pin);
	BQ25120A_attach_fault_cb(&bq_params, fault_cb);

	BQ25120A_set_LDO_voltage(&bq_params, 2.8f);
	BQ25120A_enable_LDO(&bq_params, true);
	BQ25120A_set_SYS_voltage(&bq_params, 3.3f);
	BQ25120A_set_precharge_current(&bq_params, _TARGET_PRECHARGE_CURRENT_MA);
	BQ25120A_set_battery_voltage(&bq_params, _TARGET_CHARGE_VOLTAGE);
	BQ25120A_set_charge_current(&bq_params, _TARGET_CHARGE_CURRENT_MA);
	BQ25120A_SetSafetyTimerTime(&bq_params, BQ25120A_SAFETY_TIMER_6_HOURS);

	BQ25120A_enable_button_long_press_to_enable_ship_mode(&bq_params, true);

	// check for previously logged errors
	// do not include undervolt and power not good at this point as wireless power has not been disabled
	const BQ25120A_fault_t STARTUP_FAULTS = BQ25120A_get_fault(&bq_params) & ~(BQ25120A_FAULT_VIN_UNDER_VOLTAGE | BQ25120A_FAULT_POWER_NOT_GOOD);

	CHARGER_ERROR_DBG("CHGC: Startup faults - 0x%04X\n\r", STARTUP_FAULTS);
//
//	if (BQ25120A_FAULT_NONE != STARTUP_FAULTS)
//	{
//		// there is a fault
//		// disable wireless power
//		DisableWirelessPower();
//
//		return;
//	}

	// set up the charger requests variables
	for (uint8_t i = 0; i < CHGC_MAX_CHARGER_REQUESTS; ++i)
	{
		_charger_requests[i] = DefaultChargerRequestFunction;
	}

	_charger_request_ip = 0;
	_charger_request_op = 0;
	_charger_request_count = 0;

	// TODO, debounce the charger interrupt signal

	// no faults, so enable wireless power, charging will be enabled when wireless power is detected
	EnableWirelessPower();

	_charger_state = CHGC_STATE_NOT_ON_CHARGER;
}

/**
 * @brief Function to query whether charging is active
 * @return true if charging is active
 * @return false if charging is not active
 */
bool CHGC_ChargingIsActive(void)
{
	return _charging_active;
}

/**
 * @brief Function to query whether the device is on the charger
 * @return true if the device is on the charger
 * @return false if the device is not on the charger
 */
bool CHGC_DeviceIsOnCharger(void)
{
	return _wireless_power_active;
}

/**
 * @brief Function to read a specific register from the BQ25120A charger IC
 */
bool CHGC_ReadBQ25120ARegister(uint8_t address, uint8_t * read_val_out_ptr)
{
	return BQ25120A_read_register(&bq_params, address, read_val_out_ptr);
}

/**
 * @brief Function to write to one of the 12C registers
 * @param reg The address of the register to write
 * @param new_value the new register value to use
 * @return true if writing the register was successful
 * @return false if writing the register failed
 */
bool CHGC_WriteBQ25120ARegister(uint8_t address, uint8_t new_val)
{
	return BQ25120A_write_register(&bq_params, address, new_val);
}

/**
 * @brief Function to get the battery voltage as calculated from the BQ25120A charger IC registers
 * @return float The calculated battery voltage
 */
float CHGC_BQ25120GetCalculatedBatteryVoltage(void)
{
	return _latest_read_battery_voltage;
}

/**
 * @brief Function to power down the system
 * @return true if a request to power down the system was successfully added
 * @return false if a request to power down the system failed to be added
 */
bool CHGC_PowerDownSystem(void)
{
	return AddChargerRequestToQueue(PowerDownRequest);
}

/**
 * @brief Function to set the charger into a safe state in the event of an error
 */
void CHGC_PutChargerIntoErrorSafeState(void)
{
	// stop any charging
	DisableWirelessPower();

	// make sure the charger is powered from the battery
	SetChargerPoweredFromBattery();

	// toggle the line as we will have been in high Z mode
	HAL_Delay(100);
	HAL_GPIO_WritePin(MCU_CHG_nDISABLE_GPIO_Port, MCU_CHG_nDISABLE_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(MCU_CHG_nDISABLE_GPIO_Port, MCU_CHG_nDISABLE_Pin, GPIO_PIN_SET);
}

// Private Functions

static void DisableWirelessPower(void)
{
	CHARGER_GPIO_DBG("CHGC: disabling wireless power\n\r");
	HAL_GPIO_WritePin(MCU_WIRELESS_PWR_nEN_GPIO_Port, MCU_WIRELESS_PWR_nEN_Pin, GPIO_PIN_SET);
}

static void EnableWirelessPower(void)
{
	CHARGER_GPIO_DBG("CHGC: enabling wireless power\n\r");
	HAL_GPIO_WritePin(MCU_WIRELESS_PWR_nEN_GPIO_Port, MCU_WIRELESS_PWR_nEN_Pin, GPIO_PIN_RESET);
}


static void SetChargerPoweredFromBattery(void)
{
	if (_charger_mode == CHGC_CHARGER_POWERED_FROM_BATTERY)
	{
		// No change
		return;
	}

	CHARGER_COMMS_DBG("CHGC: Setting Powered From Battery (charging disabled)\n\r");
	HAL_GPIO_WritePin(MCU_CHG_nDISABLE_GPIO_Port, MCU_CHG_nDISABLE_Pin, GPIO_PIN_SET);

	_charger_mode = CHGC_CHARGER_POWERED_FROM_BATTERY;
}

static void SetChargerPoweredFromWirelessPower(void)
{
	if (_charger_mode == CHGC_CHARGER_POWERED_FROM_WIRELESS_POWER)
	{
		// No change
		return;
	}

	CHARGER_COMMS_DBG("CHGC: Setting Powered from wireless power (charging enabled)\n\r");
	HAL_GPIO_WritePin(MCU_CHG_nDISABLE_GPIO_Port, MCU_CHG_nDISABLE_Pin, GPIO_PIN_RESET);

	_charger_mode = CHGC_CHARGER_POWERED_FROM_WIRELESS_POWER;
}

/**
 * @brief Callback for BQ25120A fault
 * @param fault The fault that has occurred
*/
static void fault_cb(BQ25120A_fault_t fault)
{
	CHARGER_ERROR_DBG("FAULT DETECTED: 0x%04X\n\r", fault);

	_bq25120a_fault_detected = (BQ25120A_FAULT_NONE != (fault & (BQ25120A_FAULT_VIN_OVER_VOLTAGE
										  | BQ25120A_FAULT_BATTERY_UNDER_VOLTAGE
										  | BQ25120A_FAULT_BATTERY_OVER_CURRENT
										  | BQ25120A_FAULT_SAFETY_TIMER)));

	_bq25120a_temperature_too_high = (BQ25120A_FAULT_TEMPERATURE_OUT_OF_BOUNDS == (fault & BQ25120A_FAULT_TEMPERATURE_OUT_OF_BOUNDS));

	if (fault & BQ25120A_FAULT_VIN_OVER_VOLTAGE) CHARGER_ERROR_DBG("  VIN over voltage fault\n\r");
	if (fault & BQ25120A_FAULT_VIN_UNDER_VOLTAGE) CHARGER_ERROR_DBG("  VIN under voltage fault\n\r");
	if (fault & BQ25120A_FAULT_BATTERY_UNDER_VOLTAGE) CHARGER_ERROR_DBG("  Battery under voltage fault\n\r");
	if (fault & BQ25120A_FAULT_BATTERY_OVER_CURRENT) CHARGER_ERROR_DBG("  Battery over current fault\n\r");
	if (fault & BQ25120A_FAULT_POWER_NOT_GOOD) CHARGER_ERROR_DBG("  Power not good fault\n\r");
	if (fault & BQ25120A_FAULT_TEMPERATURE_OUT_OF_BOUNDS) CHARGER_ERROR_DBG("  Temperature out of bounds fault\n\r");
	if (fault & BQ25120A_FAULT_TEMPERATURE_QUITE_COOL) CHARGER_ERROR_DBG("  Temperature quite cool fault\n\r");
	if (fault & BQ25120A_FAULT_TEMPERATURE_QUITE_WARM) CHARGER_ERROR_DBG("  Temperature quite warm fault\n\r");
	if (fault & BQ25120A_FAULT_RESET_COND) CHARGER_ERROR_DBG("  Reset condition fault\n\r");
	if (fault & BQ25120A_FAULT_SAFETY_TIMER) CHARGER_ERROR_DBG("  Safety timer fault\n\r");
}

/**
 * @brief Default Charger Request Function to get the status and battery voltage
 */
static bool DefaultChargerRequestFunction(void)
{
	CHARGER_COMMS_DBG("CHGC: Default Function\n\r");

	// reset the requests successful flag
	BQ25120A_ResetRequestsSuccessfulFlag(&bq_params);

	_latest_read_battery_voltage = BQ25120A_get_battery_monitor_voltage(&bq_params, false, false, false, false);
	
	BQ25120A_watchdog_kick(&bq_params);

	return BQ25120A_AllRequestsSuccessful(&bq_params);
}

/**
 * @brief Function to add a request to the queue, behind a mutex
 * @param fptr The function to call to perform the request
 * @return true if the request was successfully added to the queue
 * @return false if the request failed to be added to the queue
 */
static bool AddChargerRequestToQueue(chgc_charger_coms_fptr fptr)
{
	const bool ADD_STATUS = AddChargerRequestToQueueUnsafe(fptr);

	return ADD_STATUS;
}

/**
 * @brief Function to add a request to the queue
 * @param fptr The function to call to perform the request
 */
static bool AddChargerRequestToQueueUnsafe(chgc_charger_coms_fptr fptr)
{
	if (_charger_request_count >= CHGC_MAX_CHARGER_REQUESTS)
	{
		return false;
	}

	_charger_requests[_charger_request_ip++] = fptr;

	if (_charger_request_ip >= CHGC_MAX_CHARGER_REQUESTS)
	{
		_charger_request_ip = 0;
	}

	_charger_request_count++;

	if (1 == _charger_request_count)
	{
		// osThreadFlagsSet(_charger_comms_thread_id, CHGC_STATUS_REQUESTS_PENDING_FLAG);
	}

	return true;
}

/**
 * Request function to perform a power down
 */
static bool PowerDownRequest(void)
{
	CHARGER_COMMS_DBG("CHGC: Attempting to power Down\n\r");
	return BQ25120A_EnterShipMode(&bq_params);
}

// /**
//  * @brief Charger Communications thread function
//  * @param argument Optional argument, not used in this case
//  */
// void ChargerCommunicationsThread(void *argument)
// {
// 	const uint32_t WATCHDOG_PERIOD = 20000;

// 	for(;;)
// 	{
// 		// osThreadFlagsWait(CHGC_STATUS_REQUESTS_PENDING_FLAG, osFlagsWaitAny, WATCHDOG_PERIOD);

// 		chgc_charger_coms_fptr request_fptr = DefaultChargerRequestFunction;
// 		bool more_requests = false;

// 		if (0 != _charger_request_count)
// 		{
// 			// there are requests to process
// 			request_fptr = _charger_requests[_charger_request_op];
// 		}

// 		CHARGER_COMMS_DBG("CHGC: processing %u requests %lu\n\r", _charger_request_count, HAL_GetTick());

// 		// call the request function
// 		if (!request_fptr())
// 		{
// 			HAL_Delay(100);
// 			// the request failed, so try again
// 			more_requests = true;

// 			CHARGER_COMMS_DBG("CHGC: request failed\n\r");
// 			// TODO - add max retries
// 			// TODO - Add toggling CD line
// 		}
// 		else
// 		{
// 			// the request was successful, so remove it from the queue
// 			if (DefaultChargerRequestFunction != request_fptr)
// 			{
// 				_charger_request_op++;
// 				if (_charger_request_op >= CHGC_MAX_CHARGER_REQUESTS)
// 				{
// 					_charger_request_op = 0;
// 				}

// 				_charger_request_count--;

// 				if (0 != _charger_request_count)
// 				{
// 					more_requests = true;
// 				}
// 			}
// 		}


// 		// re-signal if there are more requests
// 		if (more_requests)
// 		{
// 			// osThreadFlagsSet(_charger_comms_thread_id, CHGC_STATUS_REQUESTS_PENDING_FLAG);
// 		}
// 	}
// }

/**
 * @brief Function to get if the wireless power is active
 * @return true if the wireless power good pin is at the active level
 * @return false if the wireless power pin is at the inactive level
 */
static bool WirelessPowerActive(void)
{
	return (GPIO_PIN_RESET == HAL_GPIO_ReadPin(MCU_CHG_nPGD_GPIO_Port, MCU_CHG_nPGD_Pin));
}

/**
 * @brief Function called when the wireless power good state changes
 * @param state The new wireless power good state
 */
static void WirelessPowerChanged(bool state)
{
	if (state)
	{
		CHARGER_STATE_MACHINE_DBG("CHGC: Wireless Power Good\n\r");
		_wireless_power_active = true;
		// wireless power is active, so enable charging
		SetChargerPoweredFromWirelessPower();
	}
	else
	{
		CHARGER_STATE_MACHINE_DBG("CHGC: Wireless Power Not Good\n\r");
		_wireless_power_active = false;
		// wireless power is no longer active, so power the charger from the battery
		SetChargerPoweredFromBattery();
	}
}

/**
 * @brief Function to get if the charger interrupt pin is at the charging active level
 * @return true if the charger interrupt pin is at the charging active level
 * @return false if the charger interrupt pin is at the charging inactive level
 */
static bool ChargingActive(void)
{
	return (GPIO_PIN_RESET == HAL_GPIO_ReadPin(MCU_CHG_INT_GPIO_Port, MCU_CHG_INT_Pin));
}

/**
 * @brief Function called when the charging state changes
 * @param state The new charging state
 */
static void ChargingStateChanged(bool state)
{
	if (state)
	{
		CHARGER_STATE_MACHINE_DBG("CHGC: Charging power good\n\r");
		_charging_power_active = true;
	}
	else
	{
		CHARGER_STATE_MACHINE_DBG("CHGC: Charging power not good\n\r");
		_charging_power_active = false;
	}
}

static const char _state_strings[5][25] = {
	"NOT_ON_CHARGER",
	"READY_TO_CHARGE",
	"CHARGING_ACTIVE_PHASE",
	"CHARGING_INACTIVE_PHASE",
	"ERROR",
};

/**
 * @brief Status monitoring thread function
 * @param argument Optional argument, not used in this case
 */
void CHGC_ChargerStatusMonitoringThread(void)
{
	chgc_state_t last_state = CHGC_STATE_READY_TO_CHARGE;

	const uint32_t STATE_MACHINE_INTERVAL = 500;
	uint32_t total_charge_time = 0;

	// for(;;)
	{
		// HAL_Delay(STATE_MACHINE_INTERVAL);

		// check if the wireless power has changed
		static bool last_wireless_power_active = false;
		const bool current_wireless_power_active = WirelessPowerActive();
		if (current_wireless_power_active != last_wireless_power_active)
		{
			WirelessPowerChanged(current_wireless_power_active);
		}
		last_wireless_power_active = current_wireless_power_active;

		// check if the charging state has changed
		static bool last_charging_active = false;
		const bool current_charging_active = ChargingActive();
		if (current_charging_active != last_charging_active)
		{
			ChargingStateChanged(current_charging_active);
		}
		last_charging_active = current_charging_active;

		if (_charger_state != last_state)
		{
			CHARGER_STATE_MACHINE_DBG("CHGC: SM - Charger State %s\n", _state_strings[_charger_state]);

			last_state = _charger_state;
		}

		bool charge_max_time_exceeded = false;

		if (_charging_active)
		{
			total_charge_time += STATE_MACHINE_INTERVAL;

			if (total_charge_time > CHGC_MAXIMUM_CHARGE_TIME_MS)
			{
				charge_max_time_exceeded = true;
			}
		}

		// check for errors
		if (_secondary_temperature_too_high || _battery_voltage_too_high || _bq25120a_fault_detected || charge_max_time_exceeded)
		{
			CHARGER_STATE_MACHINE_DBG("CHGC: SM - Errors %u %u %u\n", _secondary_temperature_too_high, _battery_voltage_too_high, _bq25120a_fault_detected);
			// disable charging
			DisableWirelessPower();
			SetChargerPoweredFromWirelessPower();

			_charger_state = CHGC_STATE_ERROR;
			//ERR_CaptureError(ERR_CODE_CHARGER_FAULT, true);
		}

		switch (_charger_state)
		{
			case CHGC_STATE_NOT_ON_CHARGER:
			{
				if (_wireless_power_active)
				{
					_charger_state = CHGC_STATE_READY_TO_CHARGE;

					CHARGER_STATE_MACHINE_DBG("CHGC: Wireless power active\n");
				}

				break;
			}
			case CHGC_STATE_READY_TO_CHARGE:
			{
				if (!_wireless_power_active)
				{
					_charger_state = CHGC_STATE_NOT_ON_CHARGER;

					CHARGER_STATE_MACHINE_DBG("CHGC: SM - Wireless power inactive \n");

					break;
				}

				if (_charging_power_active)
				{
					CHARGER_STATE_MACHINE_DBG("CHGC: SM -  Charging active\n");
					_charging_active = true;
					total_charge_time = 0;
					LED_SetCharging(true);
					_charger_state = CHGC_STATE_CHARGING_ACTIVE_PHASE;
				}

				break;
			}
			case CHGC_STATE_CHARGING_ACTIVE_PHASE:
			{
				// has charging stopped
				if (!_charging_power_active)
				{
					BQ25120A_ResetRequestsSuccessfulFlag(&bq_params);

					// has it stopped because the temperature is too high
					if (BQ25120A_IsTemperatureTooHigh(&bq_params))
					{
						_bq25120a_temperature_too_high = true;
						CHARGER_STATE_MACHINE_DBG("CHGC: SM -  Temperature too high\n");
						// disable charging, to bring the temperature back down
						DisableWirelessPower();

						_charger_state = CHGC_STATE_CHARGING_INACTIVE_PHASE;

						break;
					}
					else if (!BQ25120A_AllRequestsSuccessful(&bq_params))
					{
						// if the temperature was too low because the request failed, then try again
						break;
					}

					CHARGER_STATE_MACHINE_DBG("CHGC: SM - Charging stopped\n");
					_charging_active = false;
					LED_SetCharging(false);
					_charger_state = CHGC_STATE_READY_TO_CHARGE;

					break;
				}

				// if there are no errors, then check if we need to go to the inactive phase
				if (_bq25120a_temperature_too_high)
				{
					CHARGER_STATE_MACHINE_DBG("CHGC: SM - Charger temperature too high\n");
					// disable charging, to bring the temperature back down
					DisableWirelessPower();

					_charger_state = CHGC_STATE_CHARGING_INACTIVE_PHASE;

					break;
				}

				break;
			}
			case CHGC_STATE_CHARGING_INACTIVE_PHASE:
			{
				// Charging will have stopped at this point, so do not check this

				// if there are no errors, then check if we need to go back to the active phase
				if (!_bq25120a_temperature_too_high)
				{
					CHARGER_STATE_MACHINE_DBG("CHGC: SM - Charger temperature OK\n");

					// enable charging again now the temperature is within normal bounds
					EnableWirelessPower();

					_charger_state = CHGC_STATE_CHARGING_ACTIVE_PHASE;

					// wait 1000mS for the wireless power signal and the charging state to update
					// HAL_Delay(1000);

					break;
				}

				break;
			}
			case CHGC_STATE_ERROR:
			{

				break;
			}
			default:
				break;
		}
	}
}
