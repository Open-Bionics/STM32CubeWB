/*
 * BQ25120A_Battery_Charger.h
 *
 * Created on: March 7th, 2024
 * Author: Olly McBride
 */

#ifndef INC_BQ25120A_BATTERY_CHARGER_H_
#define INC_BQ25120A_BATTERY_CHARGER_H_

#include <stdbool.h>
#include <stdint.h>

// DEBUG
#include "stm32wbxx_hal.h"

/**
 * @brief   Delay for a number of milliseconds
 * @param   ms: number of milliseconds to delay
*/
#ifndef BQ25120A_delay_ms
#define BQ25120A_delay_ms(ms) HAL_Delay(ms)
#endif

/** BQ25120A status enum */
typedef enum
{
    BQ25120A_STATUS_READY = 0,
    BQ25120A_STATUS_CHARGING,
    BQ25120A_STATUS_CHARGE_COMPLETE,
    BQ25120A_STATUS_FAULT,

    BQ25120A_STATUS_UNKNOWN = 0xFF
} BQ25120A_status_t;

/** BQ25120A fault enum */
typedef enum
{
    BQ25120A_FAULT_NONE                         = 0x0000,   /** No fault */
    BQ25120A_FAULT_VIN_OVER_VOLTAGE             = 0x0001,   /** VIN over voltage */
    BQ25120A_FAULT_VIN_UNDER_VOLTAGE            = 0x0002,   /** VIN under voltage. VIN_UV is set when the input falls below VSLP */
    BQ25120A_FAULT_BATTERY_UNDER_VOLTAGE        = 0x0004,   /** Battery under voltage */
    BQ25120A_FAULT_BATTERY_OVER_CURRENT         = 0x0008,   /** BAT_OCP fault */
    BQ25120A_FAULT_POWER_NOT_GOOD               = 0x0010,   /** PG pulls to GND when VIN is above V(BAT) + VSLP and less that VOVP. PG is high-impedance when the input power is not within specified limits */
    BQ25120A_FAULT_TEMPERATURE_OUT_OF_BOUNDS    = 0x0020,   /** TS temp < TCOLD or TS temp > THOT (Charging suspended) */
    BQ25120A_FAULT_TEMPERATURE_QUITE_COOL       = 0x0040,   /** TCOOL > TS temp > TCOLD (Charging current reduced by half) */
    BQ25120A_FAULT_TEMPERATURE_QUITE_WARM       = 0x0080,   /** TWARM < TS temp < THOT (Charging voltage reduced by 140 mV) */
    BQ25120A_FAULT_RESET_COND                   = 0x0100,   /** Indicates when the device meets the RESET conditions */
    BQ25120A_FAULT_SAFETY_TIMER                 = 0x0200,   /** Safety timer fault */

    BQ25120A_FAULT_UNKNOWN                      = 0x1000,   /** Unknown fault */

} BQ25120A_fault_t;

/**
 * @brief Enumeration of possible safety timer time limits
 */
typedef enum _BQ25120A_SAFETY_TIMER_LIMIT_TIME
{
	BQ25120A_SAFETY_TIMER_30_MINUTES,
	BQ25120A_SAFETY_TIMER_1_HOURS,
	BQ25120A_SAFETY_TIMER_3_HOURS,
	BQ25120A_SAFETY_TIMER_6_HOURS,
	BQ25120A_SAFETY_TIMER_9_HOURS,
	BQ25120A_SAFETY_TIMER_18_HOURS,
	BQ25120A_SAFETY_TIMER_DISABLED,
}BQ25120A_safety_timer_time_t;

/**< Struct of BQ25120A parameters */
typedef struct _BQ25120A_params_t
{
    GPIO_TypeDef* INT_port;     /** INT pin port */
    uint16_t INT_pin;           /** INT pin number */
    GPIO_TypeDef* PG_port;      /** PG pin port */
    uint16_t PG_pin;            /** PG pin number */
    GPIO_TypeDef* CD_port;      /** CD pin port */
    uint16_t CD_pin;            /** CD pin number */

    float VBATREG_voltage;      /** VBATREG voltage in volts */
    bool charging_enabled;      /** Charging enabled */
    bool SYS_enabled;           /** SYS enabled */
    bool LDO_enabled;           /** LDO enabled */

    void (*fault_cb_fptr)(BQ25120A_fault_t fault);    /** Fault callback function pointer */

    bool requests_successful;
} BQ25120A_params_t;

extern bool BQ25120A_init(BQ25120A_params_t* params, GPIO_TypeDef* INT_port, uint16_t INT_pin, GPIO_TypeDef* PG_port, uint16_t PG_pin, GPIO_TypeDef* CD_port, uint16_t CD_pin);
extern void BQ25120A_attach_fault_cb(BQ25120A_params_t* params, void (*fault_cb_fptr)(BQ25120A_fault_t fault));

extern BQ25120A_status_t BQ25120A_get_status(BQ25120A_params_t* params);
extern bool BQ25120A_IsTemperatureTooHigh(BQ25120A_params_t* params);
extern bool BQ25120A_get_timer_status(BQ25120A_params_t* params);
extern bool BQ25120A_get_VINDPM_status(BQ25120A_params_t* params);
extern bool BQ25120A_get_CD_status(BQ25120A_params_t* params);
extern bool BQ25120A_get_SW_status(BQ25120A_params_t* params);
extern bool BQ25120A_get_Vin_is_present(BQ25120A_params_t* params);
extern bool BQ25120A_watchdog_kick(BQ25120A_params_t* params);
extern BQ25120A_fault_t BQ25120A_get_fault(BQ25120A_params_t* params);

extern bool BQ25120A_enable_charging(BQ25120A_params_t* params, bool enable);
extern bool BQ25120A_enable_SYS(BQ25120A_params_t* params, bool enable);
extern bool BQ25120A_enable_LDO(BQ25120A_params_t* params, bool enable);
extern bool BQ25120A_enable_high_impedance_mode(BQ25120A_params_t* params, bool enable);
extern bool BQ25120A_enable_temperature_sensor(BQ25120A_params_t* params, bool enable);
extern bool BQ25120A_enable_button_long_press_to_enable_ship_mode(BQ25120A_params_t* params, bool enable);

extern float BQ25120A_set_charge_current(BQ25120A_params_t* params, float target_mA);
extern float BQ25120A_set_precharge_current(BQ25120A_params_t* params, float target_mA);
extern float BQ25120A_set_battery_voltage(BQ25120A_params_t* params, float target_V);
extern float BQ25120A_set_SYS_voltage(BQ25120A_params_t* params, float target_V);
extern float BQ25120A_set_LDO_voltage(BQ25120A_params_t* params, float target_V);
extern float BQ25120A_set_battery_UVLO_voltage(BQ25120A_params_t* params, float target_V);
extern float BQ25120A_set_input_current_limit(BQ25120A_params_t* params, float target_mA);
extern float BQ25120A_set_manual_reset_wake_1_timer(BQ25120A_params_t* params, float time_ms);
extern float BQ25120A_set_manual_reset_wake_2_timer(BQ25120A_params_t* params, float time_ms);
extern float BQ25120A_set_manual_reset_timer_adjustment(BQ25120A_params_t* params, float time_s);

extern float BQ25120A_get_battery_monitor_voltage(BQ25120A_params_t* params, bool en_hi_z, bool pause_SYS, bool pause_LDO, bool pause_charging);

extern bool BQ25120A_read_register(BQ25120A_params_t* params, uint8_t reg, uint8_t* data_out_ptr);
extern bool BQ25120A_write_register(BQ25120A_params_t* params, uint8_t reg, uint8_t new_value);

extern bool BQ25120A_EnterShipMode(BQ25120A_params_t * params);

extern bool BQ25120A_SetSafetyTimerTime(BQ25120A_params_t * params, BQ25120A_safety_timer_time_t time);

extern void BQ25120A_ResetRequestsSuccessfulFlag(BQ25120A_params_t* params);
extern bool BQ25120A_AllRequestsSuccessful(BQ25120A_params_t* params);

#endif // INC_BQ25120A_BATTERY_CHARGER_H_

