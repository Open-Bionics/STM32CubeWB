/*
 * BQ25120A_Battery_Charger.h
 *
 * Created on: March 7th, 2024
 * Author: Olly McBride
 */

#include <string.h>                 // memcpy()
#include "BQ25120A_Battery_Charger.h"
#include "BQ25120A_reg.h"

#define BQ25120A_MAX_TX_BUF_LEN 16

extern I2C_HandleTypeDef hi2c1;

static void NullFaultHandler(BQ25120A_fault_t fault)
{
	(void)fault;

	// intentionally do nothing
}

/**
 *	@brief Function to write a register on the I2C device
 *	@param device_address The address of the device to write to
 *	@param register_addr The address of the register to write to
 *	@param data_ptr A pointer to the data to write
 *	@param data_length The length of the data to write
 *	@param timeout The timeout to use
 *	@param true if writing the register was successful
 *	@param false if writing the register failed
 */
bool I2CP_WriteToRegister(uint8_t device_address, uint8_t register_addr,  uint8_t * const data_ptr, uint16_t data_length, uint32_t timeout)
{
	// Exit early if i2c is not configured, or if parameters are invalid
	if (NULL == data_ptr) { return false; }
	if (0 == data_length) { return false; }

	const HAL_StatusTypeDef RESULT = HAL_I2C_Mem_Write(&hi2c1, device_address, register_addr, I2C_MEMADD_SIZE_8BIT, data_ptr, data_length, timeout);

	return (HAL_OK == RESULT);
}

/**
 *	@brief Function to read a register on the I2C device
 *	@param device_address The address of the device to read from
 *	@param register_addr The address of the register to read from
 *	@param data_ptr A pointer to the buffer to read the data into
 *	@param data_length The length of the read buffer
 *	@param timeout The timeout to use
 *	@param true if reading the register was successful
 *	@param false if reading the register failed
 */
bool I2CP_ReadFromRegister(uint8_t device_address, uint8_t register_addr, uint8_t * const data_ptr, uint16_t data_length, uint32_t timeout)
{
	// Exit early if i2c is not configured, or if parameters are invalid
	if (NULL == data_ptr) { return false; }
	if (0 == data_length) { return false; }

	const HAL_StatusTypeDef RESULT = HAL_I2C_Mem_Read(&hi2c1, device_address, register_addr, I2C_MEMADD_SIZE_8BIT, data_ptr, data_length, timeout);

	return (HAL_OK == RESULT);
}


/**
 * @brief Write to the Chip Disable (CD) pin
 * 
 * @param params The BQ25120A_params_t struct
 * @param state The state to write to the CD pin
*/
static void BQ25120A_CD_pin_write(BQ25120A_params_t* params, bool state)
{
	//CHARGER_DBG("    Setting Charger Pin %u\n\r", state);
    HAL_GPIO_WritePin(params->CD_port, params->CD_pin, state);
}

/**
 * @brief Read the !Power Good pin
 * 
 * @param params The BQ25120A_params_t struct
 * @returns The state of the !Power Good pin
*/
static bool BQ25120A_PG_pin_read(BQ25120A_params_t* params)
{
    return HAL_GPIO_ReadPin(params->PG_port, params->PG_pin);
}

/**
 * @brief Initialise the BQ25120A
 * 
 * @param params The BQ25120A_params_t struct
 * @param INT_port The GPIO port for the INT pin
 * @param INT_pin The GPIO pin for the INT pin
 * @param PG_port The GPIO port for the PG pin
 * @param PG_pin The GPIO pin for the PG pin
 * @param CD_port The GPIO port for the CD pin
 * @param CD_pin The GPIO pin for the CD pin
 * 
 * @return true If the initialisation was successful
*/
bool BQ25120A_init(BQ25120A_params_t* params, GPIO_TypeDef* INT_port, uint16_t INT_pin, GPIO_TypeDef* PG_port, uint16_t PG_pin, GPIO_TypeDef* CD_port, uint16_t CD_pin)
{
    if (!params)
    {
    	return false;
    }

	params->fault_cb_fptr = NullFaultHandler;

	params->INT_port = INT_port;
	params->INT_pin = INT_pin;
	params->PG_port = PG_port;
	params->PG_pin = PG_pin;
	params->CD_port = CD_port;
	params->CD_pin = CD_pin;

	params->VBATREG_voltage = BQ25120A_VBREG_DEFAULT_VOLTAGE;
	params->charging_enabled = false;
	params->LDO_enabled = false;
	params->SYS_enabled = true;

	params->requests_successful = true;

    return true;
}

/**
 * @brief Attach a fault callback
 * 
 * @param params The BQ25120A_params_t struct
 * @param fault_cb_fptr The callback function pointer
*/
void BQ25120A_attach_fault_cb(BQ25120A_params_t* params, void (*fault_cb_fptr)(BQ25120A_fault_t fault))
{
    if (params)
    {
        params->fault_cb_fptr = (NULL == fault_cb_fptr) ? NullFaultHandler : fault_cb_fptr;
    }
}

/**
 * @brief Get the charger status
 * 
 * @param params The BQ25120A_params_t struct
 * @return BQ25120A_status_t The charger status
*/
BQ25120A_status_t BQ25120A_get_status(BQ25120A_params_t* params)
{
    BQ25120A_status_t status = BQ25120A_STATUS_UNKNOWN;

    // request a read
    union BQ25120A_STATUS_REG_t reg;
    if (BQ25120A_read_register(params, BQ25120A_REG_STATUS, &reg.reg))
    {
        switch (reg.STAT)
        {
            case BQ25120A_STATUS_READY:
            case BQ25120A_STATUS_CHARGING:
            case BQ25120A_STATUS_CHARGE_COMPLETE:
            case BQ25120A_STATUS_FAULT:
                status = reg.STAT;
                break;
            default:
                status = BQ25120A_STATUS_UNKNOWN;
                break;
        }
    }

    return status;
}

/**
 * @brief Function to get whether the devices is over temperature
 * @param params the BQ25120A parameters
 * @return true if the device is over temperature
 * @return false if the device is in temperature range
 */
bool BQ25120A_IsTemperatureTooHigh(BQ25120A_params_t* params)
{
    // read the TS Control register
	union BQ25120A_TS_CONTROL_REG_t reg;
	BQ25120A_read_register(params, BQ25120A_REG_TS_CONTROL, &reg.reg);

	return (0x01 == reg.TS_FAULT);
}

/**
 * @brief Get the Timer status
 * 
 * @param params The BQ25120A_params_t struct
 * @return Timer status
*/
bool BQ25120A_get_timer_status(BQ25120A_params_t* params)
{
    bool status = false;

    // request a read
    union BQ25120A_STATUS_REG_t reg;
    if (BQ25120A_read_register(params, BQ25120A_REG_STATUS, &reg.reg))
    {
        status = reg.TIMER;
    }

    return status;
}

/**
 * @brief Get the VINDPM status (Input Voltage Dynamic Power Management)
 * 
 * @param params The BQ25120A_params_t struct
 * @return VINDPM status
*/
bool BQ25120A_get_VINDPM_status(BQ25120A_params_t* params)
{
    bool status = false;

    // request a read
    union BQ25120A_STATUS_REG_t reg;
    if (BQ25120A_read_register(params, BQ25120A_REG_STATUS, &reg.reg))
    {
        status = reg.VINDPM_STAT;
    }

    return status;
}

/**
 * @brief Get the Chip Disable status
 * 
 * @param params The BQ25120A_params_t struct
 * @return CD status
*/
bool BQ25120A_get_CD_status(BQ25120A_params_t* params)
{
    bool status = false;

    // request a read
    union BQ25120A_STATUS_REG_t reg;
    if (BQ25120A_read_register(params, BQ25120A_REG_STATUS, &reg.reg))
    {
        status = reg.CD_STAT;
    }

    return status;
}

/**
 * @brief Get the SW status (Inductor Connection)
 * 
 * @param params The BQ25120A_params_t struct
 * @return SW status
*/
bool BQ25120A_get_SW_status(BQ25120A_params_t* params)
{
    bool status = false;

    // request a read
    union BQ25120A_STATUS_REG_t reg;
    if (BQ25120A_read_register(params, BQ25120A_REG_STATUS, &reg.reg))
    {
        status = reg.SYS_EN_STAT;
    }

    return status;
}

/**
 * @brief Get whether VIN is present
 * 
 * @param params The BQ25120A_params_t struct
 * @return true If VIN is present
*/
bool BQ25120A_get_Vin_is_present(BQ25120A_params_t* params)
{
    bool Vin_present = false;

    if (params)
    {
        // !PG = VIN > UVLO
        Vin_present = (false == HAL_GPIO_ReadPin(params->PG_port, params->PG_pin));
    }

    return Vin_present;
}

/**
 * @brief Kick the watchdog by checking for faults
 * 
 * @param params The BQ25120A_params_t struct
 * @return true If the watchdog was kicked successfully
*/
bool BQ25120A_watchdog_kick(BQ25120A_params_t* params)
{
    if (NULL == params)
    {
    	return false;
    }

    BQ25120A_fault_t fault = BQ25120A_get_fault(params);

	params->fault_cb_fptr(fault);

    return true;
}

/**
 * @brief Get whether a fault has occurred
 * 
 * @param params The BQ25120A_params_t struct
 * @return BQ25120A_fault_t The fault that has occured
*/
BQ25120A_fault_t BQ25120A_get_fault(BQ25120A_params_t* params)
{
    BQ25120A_fault_t fault = BQ25120A_FAULT_NONE;

    if (params)
    {
        // read the STATUS register
        {
            union BQ25120A_STATUS_REG_t reg;
            if (BQ25120A_read_register(params, BQ25120A_REG_STATUS, &reg.reg))
            {
                if (reg.RESET_FAULT)    fault |= BQ25120A_FAULT_RESET_COND;
                if (reg.TIMER)          fault |= BQ25120A_FAULT_SAFETY_TIMER;
            }
        }

        // read the FAULT register
        {
            union BQ25120A_FAULT_REG_t reg;
            if (BQ25120A_read_register(params, BQ25120A_REG_FAULT, &reg.reg))
            {
                if (reg.VIN_OV)     fault |= BQ25120A_FAULT_VIN_OVER_VOLTAGE;
                if (reg.VIN_UV)     fault |= BQ25120A_FAULT_VIN_UNDER_VOLTAGE;
                if (reg.BAT_UVLO)   fault |= BQ25120A_FAULT_BATTERY_UNDER_VOLTAGE;
                if (reg.BAT_OCP)    fault |= BQ25120A_FAULT_BATTERY_OVER_CURRENT;
            }
        }

        // read the TS Control register
        {
            union BQ25120A_TS_CONTROL_REG_t reg;
            if (BQ25120A_read_register(params, BQ25120A_REG_TS_CONTROL, &reg.reg))
            {
                switch (reg.TS_FAULT)
                {
                    case 0x01: fault |= BQ25120A_FAULT_TEMPERATURE_OUT_OF_BOUNDS;  break;
                    case 0x02: fault |= BQ25120A_FAULT_TEMPERATURE_QUITE_COOL;     break;
                    case 0x03: fault |= BQ25120A_FAULT_TEMPERATURE_QUITE_WARM;     break;
                }
            }
        }
            
        // read the !Power good pin
        if (BQ25120A_PG_pin_read(params)) fault |= BQ25120A_FAULT_POWER_NOT_GOOD;

    }

    return fault;
}

/**
 * @brief Enable or disable charging
 * 
 * @param params The BQ25120A_params_t struct
 * @param enable true to enable charging, false to disable
 * @return true If the operation was successful
*/
bool BQ25120A_enable_charging(BQ25120A_params_t* params, bool enable)
{
    bool success = false;

    if (params)
    {
        // read the existing values from the register
        union BQ25120A_FAST_CHARGE_REG_t reg;
        if (BQ25120A_read_register(params, BQ25120A_REG_FAST_CHARGE, &reg.reg))
        {
            reg.CE = !enable;   // CE is active low

            if (BQ25120A_write_register(params, BQ25120A_REG_FAST_CHARGE, reg.reg))
            {
                params->charging_enabled = enable;
                success = true;
            }
        }
    }

    return success;
}

/**
 * @brief Enable or disable the SYS output
 * 
 * @param params The BQ25120A_params_t struct
 * @param enable true to enable SYS output, false to disable
 * @return true If the operation was successful
*/
bool BQ25120A_enable_SYS(BQ25120A_params_t* params, bool enable)
{
    bool success = false;

    if (params)
    {
        // read the existing values from the register
        union BQ25120A_SYS_VOUT_REG_t reg;
        if (BQ25120A_read_register(params, BQ25120A_REG_SYS_VOUT, &reg.reg))
        {
            reg.EN_SYS_OUT = enable;

            if (BQ25120A_write_register(params, BQ25120A_REG_SYS_VOUT, reg.reg))
            {
                params->SYS_enabled = enable;
                success = true;
            }
        }
    }

    return success;
}

/**
 * @brief Enable or disable the LDO output
 * 
 * @param params The BQ25120A_params_t struct
 * @param enable true to enable LDO, false to disable
 * @return true If the operation was successful
*/
bool BQ25120A_enable_LDO(BQ25120A_params_t* params, bool enable)
{
    bool success = false;

    if (params)
    {
        // read the existing values from the register
        union BQ25120A_LDO_CONTROL_REG_t reg;
        if (BQ25120A_read_register(params, BQ25120A_REG_LDO_CONTROL, &reg.reg))
        {
            reg.EN_LS_LDO = enable;

            if (BQ25120A_write_register(params, BQ25120A_REG_LDO_CONTROL, reg.reg))
            {
                params->LDO_enabled = enable;
                success = true;
            }
        }
    }

    return success;
}

/**
 * @brief Enable or disable High Impedance mode for reading the battery voltage with no load
 * 
 * @param params The BQ25120A_params_t struct
 * @param enable true to enable Hi-Z mode, false to disable
 * @return true If the operation was successful
*/
bool BQ25120A_enable_high_impedance_mode(BQ25120A_params_t* params, bool enable)
{
    bool success = false;

    /**
     * When the HZ_MODE bit is written by the host, the I2C interface is disabled if only battery is present. To resume I2C, the CD 
     * pin must be toggled.
    */

    if (params)
    {
        // if we are coming out of Hi-Z mode, toggle the CD pin to re-enable the I2C interface
        if (!enable)
        {
            BQ25120A_CD_pin_write(params, true);
            BQ25120A_delay_ms(1);         // 100us recommended deglitch time, use min tick time of 1mS
            BQ25120A_CD_pin_write(params, false);
        }

        // read the existing values from the register
        union BQ25120A_FAST_CHARGE_REG_t reg;
        if (BQ25120A_read_register(params, BQ25120A_REG_FAST_CHARGE, &reg.reg))
        {
            reg.HZ_MODE = enable;
            if (BQ25120A_write_register(params, BQ25120A_REG_FAST_CHARGE, reg.reg))
            {
                success = true;
            }
        }
    }

    return success;
}

/**
 * @brief Enable or disable the temperature sensor
 * 
 * @param params The BQ25120A_params_t struct
 * @param enable true to enable the temperature sensor, false to disable
 * @return true If the operation was successful
*/
bool BQ25120A_enable_temperature_sensor(BQ25120A_params_t* params, bool enable)
{
    bool success = false;

    if (params)
    {
        // read the existing values from the register
        union BQ25120A_TS_CONTROL_REG_t reg;
        if (BQ25120A_read_register(params, BQ25120A_REG_TS_CONTROL, &reg.reg))
        {
            reg.TS_EN = enable;
            if (BQ25120A_write_register(params, BQ25120A_REG_TS_CONTROL, reg.reg))
            {
                success = true;
            }
        }
    }

    return success;
}

/**
 * @brief Enable or disable the button long press to enable ship mode
 * 
 * @param params The BQ25120A_params_t struct
 * @param enable true to enable the button long press to enable ship mode, false to disable
 * @return true If the operation was successful
*/
bool BQ25120A_enable_button_long_press_to_enable_ship_mode(BQ25120A_params_t* params, bool enable)
{
    bool success = false;

    if (params)
    {
        // read the existing values from the register
        union BQ25120A_BUTTON_REG_t reg;
        if (BQ25120A_read_register(params, BQ25120A_REG_BUTTON, &reg.reg))
        {
            reg.MRREC = !enable;        // 0 – After Reset, device enters Ship mode, 1 – After Reset, device enters Hi-Z Mode
            if (BQ25120A_write_register(params, BQ25120A_REG_BUTTON, reg.reg))
            {
                success = true;
            }
        }
    }

    return success;
}

/**
 * @brief Set the charge current
 * 
 * @param params The BQ25120A_params_t struct
 * @param target_mA The target current in mA (5 mA - 300 mA)
 * @return float The actual current that was set, in mA
 */
float BQ25120A_set_charge_current(BQ25120A_params_t* params, float target_mA)
{
    float actual_mA = 0.0f;

    if (params)
    {
       uint8_t range = 0xFF;

        // if the target current is within RANGE_0 (including between range_0_upper range_1_lower, as the chip auto clamps to 35mA if out of range)
        if ((target_mA >= BQ25120A_FAST_CHARGE_RANGE_0_LOWER_mA) && (target_mA < BQ25120A_FAST_CHARGE_RANGE_1_LOWER_mA))
        {
            range = 0;
        }
        // else if the target current is within RANGE_1
        else if ((target_mA >= BQ25120A_FAST_CHARGE_RANGE_1_LOWER_mA) && (target_mA <= BQ25120A_FAST_CHARGE_RANGE_1_UPPER_mA))
        {
            range = 1;
        }

        // if a valid range was detected
        if (0xFF != range)
        {
            // determine the step size for this range
            const float step  = (0 == range) ? BQ25120A_FAST_CHARGE_RANGE_0_STEP_mA  : BQ25120A_FAST_CHARGE_RANGE_1_STEP_mA;
            const float lower = (0 == range) ? BQ25120A_FAST_CHARGE_RANGE_0_LOWER_mA : BQ25120A_FAST_CHARGE_RANGE_1_LOWER_mA;

            union BQ25120A_FAST_CHARGE_REG_t reg;
            if (BQ25120A_read_register(params, BQ25120A_REG_FAST_CHARGE, &reg.reg))
            {
            	// update the charge current
            	reg.ICHRG_RANGE = range;
            	reg.ICHRG = (uint8_t)((target_mA - lower) / step);

                if (BQ25120A_write_register(params, BQ25120A_REG_FAST_CHARGE, reg.reg))
				{
					// convert the ICHRG back to a current to be returned
					actual_mA = lower + ((float)reg.ICHRG * step);
				}
            }
        }
    }

    return actual_mA;
}

/**
 * @brief Set the precharge current and enable charging
 *
 * @param params The BQ25120A_params_t struct
 * @param target_mA The target current in mA (5 mA - 300 mA)
 * @return float The actual current that was set, in mA
 */
float BQ25120A_set_precharge_current(BQ25120A_params_t* params, float target_mA)
{
    float actual_mA = 0.0f;

    if (params)
    {
       uint8_t range = 0xFF;

        // if the target current is within RANGE_0 (including between range_0_upper range_1_lower, as the chip auto clamps to 35mA if out of range)
        if ((target_mA >= BQ25120A_PRE_CHARGE_RANGE_0_LOWER_mA) && (target_mA < BQ25120A_PRE_CHARGE_RANGE_1_LOWER_mA))
        {
            range = 0;
        }
        // else if the target current is within RANGE_1
        else if ((target_mA >= BQ25120A_PRE_CHARGE_RANGE_1_LOWER_mA) && (target_mA <= BQ25120A_PRE_CHARGE_RANGE_1_UPPER_mA))
        {
            range = 1;
        }

        // if a valid range was detected
        if (0xFF != range)
        {
            // determine the step size for this range
            const float step  = (0 == range) ? BQ25120A_PRE_CHARGE_RANGE_0_STEP_mA  : BQ25120A_PRE_CHARGE_RANGE_1_STEP_mA;
            const float lower = (0 == range) ? BQ25120A_PRE_CHARGE_RANGE_0_LOWER_mA : BQ25120A_PRE_CHARGE_RANGE_1_LOWER_mA;

            union BQ25120A_PRE_CHARGE_REG_t reg;
            if (BQ25120A_read_register(params, BQ25120A_REG_TERMINATION, &reg.reg))
            {
            	// update the precharge current
            	reg.ICHRG_RANGE = range;
            	reg.ICHRG = (uint8_t)((target_mA - lower) / step);

                if (BQ25120A_write_register(params, BQ25120A_REG_TERMINATION, reg.reg))
                {
                    // convert the ICHRG back to a current to be returned
                    actual_mA = lower + ((float)reg.ICHRG * step);
                }
            }
        }
    }

    return actual_mA;
}

/**
 * @brief Set the battery voltage
 * 
 * @param params The BQ25120A_params_t struct
 * @param target_V The target voltage in V (3.6V - 4.65V)
 * @return float The actual voltage that was set, in V
 */
float BQ25120A_set_battery_voltage(BQ25120A_params_t* params, float target_V)
{
    float actual_V = 0.0f;

    if (params)
    {
        // Ensure target voltage is within the valid range (0.8 V to 3.9 V)
        if (target_V < BQ25120A_BATT_VOLTAGE_MIN_V) target_V = BQ25120A_BATT_VOLTAGE_MIN_V;
        if (target_V > BQ25120A_BATT_VOLTAGE_MAX_V) target_V = BQ25120A_BATT_VOLTAGE_MAX_V;

        // Construct the register value
        union BQ25120A_BATT_VOLTAGE_REG_t reg = 
        {
            .VBREG = (uint8_t)((target_V - BQ25120A_BATT_VOLTAGE_MIN_V) / BQ25120A_BATT_VOLTAGE_STEP_V)
        };

        // Write to the device
        if (BQ25120A_write_register(params, BQ25120A_REG_BATT_VOLTAGE, reg.reg))
        {
            // Calculate the actual voltage set, for verification or further use
            actual_V = BQ25120A_BATT_VOLTAGE_MIN_V + ((float)reg.VBREG * BQ25120A_BATT_VOLTAGE_STEP_V);
        }
    }

    return actual_V;
}

/**
 * @brief Set the system output voltage (switch-mode)
 * 
 * @param params The BQ25120A_params_t struct
 * @param target_V The target voltage in V (1.1 V - 3.3 V)
 * @return float The actual voltage that was set, in V
 */
float BQ25120A_set_SYS_voltage(BQ25120A_params_t* params, float target_V)
{
    float actual_V = 0.0f;

    if (params)
    {
        // if the target voltage is within the valid range
        if ((target_V >= SYS_SEL_LOOKUP_TABLE[0].voltage) && (target_V <= SYS_SEL_LOOKUP_TABLE[SYS_SEL_LOOKUP_TABLE_N_ENTRIES-1].voltage))
        {
            // search for the closest voltage setting in the lookup table
            uint8_t i;
            for (i = 0; i < SYS_SEL_LOOKUP_TABLE_N_ENTRIES - 1; i++)
            {
                if (SYS_SEL_LOOKUP_TABLE[i].voltage >= target_V)
                {
                    break;
                }
            }

            // If the found voltage is greater than target, use the previous one
            if (SYS_SEL_LOOKUP_TABLE[i].voltage > target_V)
            {
                i -= 1;
            }

            union BQ25120A_SYS_VOUT_REG_t reg;
            if (BQ25120A_read_register(params, BQ25120A_REG_SYS_VOUT, &reg.reg))
            {
            	reg.SYS_VOUT = SYS_SEL_LOOKUP_TABLE[i].SYS_VOUT;
            	reg.SYS_SEL = SYS_SEL_LOOKUP_TABLE[i].SYS_SEL;
            	reg.EN_SYS_OUT = true;

                if (BQ25120A_write_register(params, BQ25120A_REG_SYS_VOUT, reg.reg))
                {
                    actual_V = SYS_SEL_LOOKUP_TABLE[i].voltage;
                }
            }
        }
    }

    return actual_V;
}

/**
 * @brief Set the LDO voltage (linear regulator)
 * 
 * @param params The BQ25120A_params_t struct
 * @param target_V The target voltage in V (0.8 V - 3.9V)
 * @return float The actual voltage that was set, in V
 */
float BQ25120A_set_LDO_voltage(BQ25120A_params_t* params, float target_V)
{
    float actual_V = 0.0f;

    if (params)
    {
        // Ensure target voltage is within the valid range (0.8 V to 3.9 V)
        if (target_V < BQ25120A_LDO_MIN_V) target_V = BQ25120A_LDO_MIN_V;
        if (target_V > BQ25120A_LDO_MAX_V) target_V = BQ25120A_LDO_MAX_V;

        union BQ25120A_LDO_CONTROL_REG_t reg;
        if (BQ25120A_read_register(params, BQ25120A_REG_LDO_CONTROL, &reg.reg))
        {
        	reg.LS_LDO = (uint8_t)((target_V - BQ25120A_LDO_MIN_V) / BQ25120A_LDO_STEP_V);
        	reg.EN_LS_LDO = true;

            // Write to the device
            if (BQ25120A_write_register(params, BQ25120A_REG_LDO_CONTROL, reg.reg))
            {
                // Calculate the actual voltage set, for verification or further use
                actual_V = BQ25120A_LDO_MIN_V + ((float)reg.LS_LDO * BQ25120A_LDO_STEP_V);
            }
        }
    }

    return actual_V;
}

/**
 * @brief Set the battery UVLO voltage (under-voltage lockout)
 * 
 * @param params The BQ25120A_params_t struct
 * @param target_V The UVLO targer voltage in V (2.2 V - 3.0V)
 * @return float The actual UVLO voltage that was set, in V
 */
float BQ25120A_set_battery_UVLO_voltage(BQ25120A_params_t* params, float target_V)
{
    float actual_V = 0.0f;

    if (params)
    {
        // if the target voltage is within the valid range
        if ((target_V >= BUVLO_LOOKUP_TABLE[0].voltage) && (target_V <= BUVLO_LOOKUP_TABLE[BUVLO_LOOKUP_TABLE_N_ENTRIES-1].voltage))
        {
            // search for the closest voltage setting in the lookup table
            uint8_t i;
            for (i = 0; i < BUVLO_LOOKUP_TABLE_N_ENTRIES - 1; i++)
            {
                if (BUVLO_LOOKUP_TABLE[i].voltage >= target_V)
                {
                    break;
                }
            }

            // If the found voltage is greater than target, use the previous one
            if (BUVLO_LOOKUP_TABLE[i].voltage > target_V)
            {
                i -= 1;
            }

            union BQ25120A_INLIM_UVLO_REG_t reg = { .BUVLO = BUVLO_LOOKUP_TABLE[i].BUVLO};
            if (BQ25120A_write_register(params, BQ25120A_REG_INLIM_UVLO, reg.reg))
            {
                actual_V = BUVLO_LOOKUP_TABLE[i].voltage;
            }
        }
    }

    return actual_V;
}

/**
 * @brief Set the input current limit
 * 
 * @param params The BQ25120A_params_t struct
 * @param target_mA The target input current limit in mA (100 mA - 400 mA)
 * @return float The actual input current limit that was set, in mA
 */
float BQ25120A_set_input_current_limit(BQ25120A_params_t* params, float target_mA)
{
    float actual_mA = 0.0f;

    if (params)
    {
        // Calculate the INLIM code based on the target input current limit
        int INLIM_code = (int)((target_mA - BQ25120A_INLIM_MIN_mA) / BQ25120A_INLIM_STEP_mA);

        // Ensure the INLIM code is within the valid range (0x00 to 0x07)
        if (INLIM_code < 0x00) INLIM_code = 0x00;
        if (INLIM_code > 0x07) INLIM_code = 0x07;

        // Construct the register value
        union BQ25120A_INLIM_UVLO_REG_t reg = 
        {
            .INLIM = (uint8_t)INLIM_code
        };

        // Write to the device
        if (BQ25120A_write_register(params, BQ25120A_REG_INLIM_UVLO, reg.reg))
        {
            // Calculate the actual current limit set, for verification or further use
            actual_mA = BQ25120A_INLIM_MIN_mA + ((float)reg.INLIM * BQ25120A_INLIM_STEP_mA);
        }
    }

    return actual_mA;
}

/**
 * @brief Set the manual reset wake 1 timer
 * 
 * @param params The BQ25120A_params_t struct
 * @param time_ms The target time in ms (80 ms or 600 ms)
 * @return float The actual time that was set, in ms
 */
float BQ25120A_set_manual_reset_wake_1_timer(BQ25120A_params_t* params, float time_ms)
{
    float actual_time_ms = 0.0f;

    if (params)
    {
        // ensure the time is within the valid range (80 ms or 600 ms)
        uint8_t bit = 0;       // default 0 (80 ms)
        if (time_ms == 600.0f)
        {
            bit = 1;
        }
        else
        {
            time_ms = 80.0f;
        }

        // read the existing values from the register
        union BQ25120A_BUTTON_REG_t reg;
        if (BQ25120A_read_register(params, BQ25120A_REG_BUTTON, &reg.reg))
        {
            reg.MRWAKE_1 = bit;        // 0 – 80 ms, 1 – 600 ms
            if (BQ25120A_write_register(params, BQ25120A_REG_BUTTON, reg.reg))
            {
                actual_time_ms = time_ms;
            }
        }
    }

    return actual_time_ms;
}

/**
 * @brief Set the manual reset wake 2 timer
 * 
 * @param params The BQ25120A_params_t struct
 * @param time_ms The target time in ms (1000 ms or 1500 ms)
 * @return float The actual time that was set, in ms
 */
float BQ25120A_set_manual_reset_wake_2_timer(BQ25120A_params_t* params, float time_ms)
{
    float actual_time_ms = 0.0f;

    if (params)
    {
        // ensure the time is within the valid range (1000 ms or 1500 ms)
        uint8_t bit = 1;       // default 1 (1500 ms)
        if (time_ms == 1000.0f)
        {
            bit = 0;
        }
        else
        {
            time_ms = 1500.0f;
        }

        // read the existing values from the register
        union BQ25120A_BUTTON_REG_t reg;
        if (BQ25120A_read_register(params, BQ25120A_REG_BUTTON, &reg.reg))
        {
            reg.MRWAKE_2 = bit;        // 0 – 1000 ms, 1 – 1500 ms
            if (BQ25120A_write_register(params, BQ25120A_REG_BUTTON, reg.reg))
            {
                actual_time_ms = time_ms;
            }
        }
    }

    return actual_time_ms;
}

/**
 * @brief Set the manual reset wake 2 timer
 * 
 * @param params The BQ25120A_params_t struct
 * @param time_ms The target time in ms (5s, 9s, 11s or 15s)
 * @return float The actual time that was set, in s
 */
float BQ25120A_set_manual_reset_timer_adjustment(BQ25120A_params_t* params, float time_s)
{
    float actual_time_s = 0.0f;

    if (params)
    {
        union BQ25120A_BUTTON_REG_t reg;
        if (BQ25120A_read_register(params, BQ25120A_REG_BUTTON, &reg.reg))
        {
            if (time_s <= 5.0f)
            {
                reg.MRRESET = 0x00;
                actual_time_s = 5.0f;
            }
            else if (time_s <= 9.0f)
            {
                reg.MRRESET = 0x01;
                actual_time_s = 9.0f;
            }
            else if (time_s <= 11.0f)
            {
                reg.MRRESET = 0x02;
                actual_time_s = 11.0f;
            }
            else
            {
                reg.MRRESET = 0x03;
                actual_time_s = 15.0f;
            }

            BQ25120A_write_register(params, BQ25120A_REG_BUTTON, reg.reg);
        }
    }

    return actual_time_s;
}

/**
 * @brief Get the battery monitor voltage
 * 
 * @details Reading the value during charge is possible, but for the most accurate battery voltage indication, it is 
 * recommended to disable charge, initiate a read, and then re-enable charge.
 * 
 * @param params The BQ25120A_params_t struct
 * 
 * @param en_hi_z true to enable Hi-Z mode whilst reading the voltage
 * @param pause_SYS true to pause SYS whilst reading the voltage
 * @param pause_LDO true to pause the LDO whilst reading the voltage
 * @param pause_charging true to pause charging while reading the voltage
 * @return float The battery voltage, in V
*/
float BQ25120A_get_battery_monitor_voltage(BQ25120A_params_t* params, bool en_hi_z, bool pause_SYS, bool pause_LDO, bool pause_charging)
{
	// TODO - improve handling of disabling / re-enabling features.

    float voltage_V = 0.0f;

    if (params)
    {
        bool resume_charging = false;
        bool resume_SYS = false;
        bool resume_LDO = false;
        bool disable_hi_z = false;

        // if we want to pause the SYS output to read the battery voltage
        if (pause_SYS && params->SYS_enabled)
        {
            BQ25120A_enable_SYS(params, false);
            resume_SYS = true;
        }

        // if we want to pause the LDO to read the battery voltage
        if (pause_LDO && params->LDO_enabled)
        {
            BQ25120A_enable_LDO(params, false);
            resume_LDO = true;
        }

        // if we want to pause charging to read the battery voltage
        if (pause_charging && params->charging_enabled)
        {
            BQ25120A_enable_charging(params, false);
            resume_charging = true;
        }

        bool pre_read_tasks_successful = true;
        if(en_hi_z)
        {
            // enable Hi-Z mode which automatically triggers a read
            pre_read_tasks_successful = BQ25120A_enable_high_impedance_mode(params, true);
            disable_hi_z = true;
        }
        else
        {
            union BQ25120A_BATT_MON_REG_t reg = { .VBMON_READ = true };
            pre_read_tasks_successful = BQ25120A_write_register(params, BQ25120A_REG_BATT_MONITOR, reg.reg);
        }

        if (pre_read_tasks_successful)
        {
            // wait 2ms
            BQ25120A_delay_ms(2);

            // read the value
            union BQ25120A_BATT_MON_REG_t reg;
            if (BQ25120A_read_register(params, BQ25120A_REG_BATT_MONITOR, &reg.reg))
            {
                // Calculate the battery voltage based on VBMON_RANGE and VBMON_TH
                float range_min = 0.6f; // 60% is the minimum range
                float range_max = 0.7f; // 70% is the next step after the minimum range
                switch (reg.VBMON_RANGE)
                {
                    case 0b00: range_min = 0.6f; range_max = 0.7f; break;
                    case 0b01: range_min = 0.7f; range_max = 0.8f; break;
                    case 0b10: range_min = 0.8f; range_max = 0.9f; break;
                    case 0b11: range_min = 0.9f; range_max = 1.0f; break;
                }

                float threshold_percentage = (reg.VBMON_TH + 1) * 0.02f; // Each step is 2%
                voltage_V = params->VBATREG_voltage * (range_min + (range_max - range_min) * threshold_percentage);
            }
        }

        // resume anything that was paused for the battery voltage measurement
        if (disable_hi_z)       BQ25120A_enable_high_impedance_mode(params, false);
        if (resume_SYS)         BQ25120A_enable_SYS(params, true);
        if (resume_LDO)         BQ25120A_enable_LDO(params, true);
        if (resume_charging)    BQ25120A_enable_charging(params, true);
    }

    return voltage_V;
}

/**
 * @brief Function to read a register from the BQ25120A charger IC
 * @param params The BQ25120A devices parameters
 * @param reg The address of the register to read
 * @param data_out_ptr The pointer of the variable to set as the read register value.
 * @param uint16_t The number of bytes read
 */
bool BQ25120A_read_register(BQ25120A_params_t* params, uint8_t reg, uint8_t* data_out_ptr)
{
	if (NULL == data_out_ptr) { return false; }
	if (reg > BQ25120A_REG_VIN_DPM) { return false; }

	volatile bool RESULT = I2CP_ReadFromRegister(BQ25120A_ADDR, reg, data_out_ptr, 1, 100);

	//CHARGER_DBG("BQ25120A: Read 0x%02X from Register 0x%02X - %s\n", *data_out_ptr, reg, RESULT ? "Successful" : "Failed");

	// update the requests successful flag
	params->requests_successful &= RESULT;

	return RESULT;
}

/**
 * @brief Function to write to one of the 12C registers
 * @param params The BQ25120A devices parameters
 * @param reg The address of the register to write
 * @param new_value the new register value to use
 * @return true if writing the register was successful
 * @return false if writing the register failed
 */
bool BQ25120A_write_register(BQ25120A_params_t* params, uint8_t reg, uint8_t new_value)
{
	if (reg > BQ25120A_REG_VIN_DPM) { return false; }

	const bool RESULT = I2CP_WriteToRegister(BQ25120A_ADDR, reg, &new_value, 1, 100);

	//CHARGER_DBG("BQ25120A: Writing 0x%02X to Register 0x%02X - %s\n", new_value, reg, RESULT ? "Successful" : "Failed");

	// update the requests successful flag
	params->requests_successful &= RESULT;

	return RESULT;
}

/**
 * @brief Function to enter ship mode immediately
 * @param params The parameters for the charger chip
 * @return true if the register was successfully written
 * @return false if there was an issue reading/writing from/to the register
 */
bool BQ25120A_EnterShipMode(BQ25120A_params_t * params)
{
    union BQ25120A_STATUS_REG_t reg;
    if (!BQ25120A_read_register(params, BQ25120A_REG_STATUS, &reg.reg))
    {
    	return false;
    }

    reg.EN_SHIPMODE = true;

    if (!BQ25120A_write_register(params, BQ25120A_REG_STATUS, reg.reg))
    {
        return false;
    }

    return true;
}

/**
 * @brief Function to set the safety timer time
 * @param params The parameters for the charger chip
 * @param time The safety timer time to set
 * @return true if the register was successfully written
 * @return false if there was an issue reading/writing from/to the register
 */
bool BQ25120A_SetSafetyTimerTime(BQ25120A_params_t * params, BQ25120A_safety_timer_time_t time)
{
    union BQ25120A_VIN_DPM_TMRS_REG_t reg;
    if (!BQ25120A_read_register(params, BQ25120A_REG_VIN_DPM, &reg.reg))
    {
    	return false;
    }

    bool invalid_time = false;

    switch (time)
    {
		case BQ25120A_SAFETY_TIMER_30_MINUTES:
		{
			reg.TMR = 0;
			reg.TMRX2_EN = 0;
			break;
		}
		case BQ25120A_SAFETY_TIMER_1_HOURS:
		{
			reg.TMR = 0;
			reg.TMRX2_EN = 1;
			break;
		}
		case BQ25120A_SAFETY_TIMER_3_HOURS:
		{
			reg.TMR = 1;
			reg.TMRX2_EN = 0;
			break;
		}
		case BQ25120A_SAFETY_TIMER_6_HOURS:
		{
			reg.TMR = 1;
			reg.TMRX2_EN = 1;
			break;
		}
		case BQ25120A_SAFETY_TIMER_9_HOURS:
		{
			reg.TMR = 2;
			reg.TMRX2_EN = 0;
			break;
		}
		case BQ25120A_SAFETY_TIMER_18_HOURS:
		{
			reg.TMR = 2;
			reg.TMRX2_EN = 1;
			break;
		}
		case BQ25120A_SAFETY_TIMER_DISABLED:
		{
			reg.TMR = 3;
			reg.TMRX2_EN = 0;
			break;
		}
		default:
			// do not modify the register if the time is not recognised
			invalid_time = true;	// set flag to indicate that the time was invalid
			break;
    }

    if (invalid_time)
    {
    	// the time was invalid, so return false
    	return false;
    }

    if (!BQ25120A_write_register(params, BQ25120A_REG_VIN_DPM, reg.reg))
    {
        return false;
    }

    return true;
}

/**
 * @brief Function to reset the requests successful flag
 * @param params The parameters to reset the flag in
 */
void BQ25120A_ResetRequestsSuccessfulFlag(BQ25120A_params_t* params)
{
	params->requests_successful = true;
}

/**
 * @brief Function to get whether all requests were successful since the last time the flag was reset
 * @param params The parameters to get the flag for
 * @return true if all requests have been successful
 * @return false if any request fails
 */
bool BQ25120A_AllRequestsSuccessful(BQ25120A_params_t* params)
{
	return params->requests_successful;
}
