/*
 * BQ25120A_reg.h
 *
 * Created on: March 6th, 2024
 * Author: Olly McBride
 */

#ifndef INC_BQ25120A_REG_H_
#define INC_BQ25120A_REG_H_

#include <stdint.h>

// I2C Address of the BQ25120A Battery Charger
#define BQ25120A_ADDR                       0xD4

// Register Map
#define BQ25120A_REG_STATUS                 0x00    /** Status Register Address */
#define BQ25120A_REG_FAULT                  0x01    /** Fault Register Address */
#define BQ25120A_REG_TS_CONTROL             0x02    /** TS Control Register Address */
#define BQ25120A_REG_FAST_CHARGE            0x03    /** Fast Charge Control Register Address */
#define BQ25120A_REG_TERMINATION            0x04    /** Termination Control Register Address */
#define BQ25120A_REG_BATT_VOLTAGE           0x05    /** Battery Voltage Control Register Address */
#define BQ25120A_REG_SYS_VOUT               0x06    /** SYS VOUT Control Register Address */
#define BQ25120A_REG_LDO_CONTROL            0x07    /** LDO Control Register Address */
#define BQ25120A_REG_BUTTON                 0x08    /** Button Register Address */
#define BQ25120A_REG_INLIM_UVLO             0x09    /** INLIM/UVLO Register Address */
#define BQ25120A_REG_BATT_MONITOR           0x0A    /** Battery Monitor Register Address */
#define BQ25120A_REG_VIN_DPM                0x0B    /** VIN DPM Register Address */


#pragma pack(push, 1)
/**< A union to represent the Status control register */
union BQ25120A_STATUS_REG_t
{
    uint8_t reg;
    struct
    {
        uint8_t SYS_EN_STAT : 1;    /** b0 */
        uint8_t CD_STAT : 1;        /** b1 */
        uint8_t VINDPM_STAT : 1;    /** b2 */
        uint8_t TIMER : 1;          /** b3 */
        uint8_t RESET_FAULT : 1;    /** b4 */
        uint8_t EN_SHIPMODE : 1;    /** b5 */
        uint8_t STAT : 2;           /** b6 - b7 */
    };
};

/**< A union to represent the Fault register */
union BQ25120A_FAULT_REG_t
{
    uint8_t reg;
    struct
    {
        uint8_t BAT_OCP_M : 1;      /** b0 */
        uint8_t BAT_UVLO_M : 1;     /** b1 */
        uint8_t VIN_UV_M : 1;       /** b2 */
        uint8_t VIN_OV_M : 1;       /** b3 */
        uint8_t BAT_OCP : 1;        /** b4 */
        uint8_t BAT_UVLO : 1;       /** b5 */
        uint8_t VIN_UV : 1;         /** b6 */
        uint8_t VIN_OV : 1;         /** b7 */
    };
};

/**< A union to represent the Temperature Sensor Control register */
union BQ25120A_TS_CONTROL_REG_t
{
    uint8_t reg;
    struct
    {
        uint8_t TIMER_M : 1;        /** b0 */
        uint8_t RESET_M : 1;        /** b1 */
        uint8_t WAKE_M : 1;         /** b2 */
        uint8_t EN_INT : 1;         /** b3 */
        uint8_t RESERVED : 1;       /** b4 */
        uint8_t TS_FAULT : 2;       /** b5-6 */
        uint8_t TS_EN : 1;          /** b7 */
    };
};

/**< A union to represent the Fast Charge control register */
union BQ25120A_FAST_CHARGE_REG_t
{
    uint8_t reg;
    struct
    {
        uint8_t HZ_MODE : 1;        /** b0 */
        uint8_t CE : 1;             /** b1 */
        uint8_t ICHRG : 5;          /** b2 - b6 */
        uint8_t ICHRG_RANGE : 1;    /** b7 */
    };
};

/**< A union to represent the Termination Pre Charge control register */
union BQ25120A_PRE_CHARGE_REG_t
{
    uint8_t reg;
    struct
    {
        uint8_t RESERVED : 1;        /** b0 */
        uint8_t CE : 1;             /** b1 */
        uint8_t ICHRG : 5;          /** b2 - b6 */
        uint8_t ICHRG_RANGE : 1;    /** b7 */
    };
};

/**< A union to represent the Battery Voltage control register */
union BQ25120A_BATT_VOLTAGE_REG_t
{
    uint8_t reg;
    struct
    {
        uint8_t RESERVED : 1;       /** b0 */
        uint8_t VBREG : 7;          /** b1 - b7 */
    };
};

/**< A union to represent the SYS VOUT control register */
union BQ25120A_SYS_VOUT_REG_t
{
    uint8_t reg;
    struct
    {
        uint8_t RESERVED : 1;       /** b0 */
        uint8_t SYS_VOUT : 4;       /** b1 - b4 */
        uint8_t SYS_SEL : 2;        /** b5 - b6 */
        uint8_t EN_SYS_OUT : 1;     /** b7 */
    };
};

/**< A union to represent the LDO control register */
union BQ25120A_LDO_CONTROL_REG_t
{
    uint8_t reg;
    struct
    {
        uint8_t MRRESET_VIN : 1;    /** b0 */
        uint8_t RESERVED : 1;       /** b1 */
        uint8_t LS_LDO : 5;         /** b2 - b6 */
        uint8_t EN_LS_LDO : 1;      /** b7 */
    };
};

/**< A union to represent the Battery Monitor register */
union BQ25120A_BATT_MON_REG_t
{
    uint8_t reg;
    struct
    {
        uint8_t RESERVED : 2;       /** b0 - b1 */
        uint8_t VBMON_TH : 3;       /** b2 - b4 */
        uint8_t VBMON_RANGE : 2;    /** b5 - b6 */
        uint8_t VBMON_READ : 1;     /** b7 */
    };
};

/**< A union to represent the Push Button Control register */
union BQ25120A_BUTTON_REG_t
{
    uint8_t reg;
    struct
    {
        uint8_t WAKE : 2;           /** b0 - b1 (Read-only) */
        uint8_t PGB_MR : 1;         /** b2 - b3 */
        uint8_t MRRESET : 2;        /** b4 */
        uint8_t MRREC : 1;          /** b5 */
        uint8_t MRWAKE_2 : 1;       /** b6 */
        uint8_t MRWAKE_1 : 1;       /** b7 */
    };
};

/**< A union to represent the Input Current Limit & UVLO register */
union BQ25120A_INLIM_UVLO_REG_t
{
    uint8_t reg;
    struct
    {
        uint8_t BUVLO : 3;          /** b0 - b2 */
        uint8_t INLIM : 3;          /** b3 - b5 */
        uint8_t RESERVED : 1;       /** b6 */
        uint8_t RESET : 1;          /** b7 */
    };
};

/**< A union to represent the VIN_DPM and timers register */
union BQ25120A_VIN_DPM_TMRS_REG_t
{
    uint8_t reg;
    struct
    {
        uint8_t RESERVED : 1;          /** b0 */
        uint8_t TMR : 2;               /** b1 - b2 */
        uint8_t TMRX2_EN : 1;          /** b3 */
        uint8_t VINDPM_VOLTAGE : 3;    /** b4 - b6 */
        uint8_t VINDPM_ON : 1;         /** b7 */
    };
};
#pragma pack(pop)


// Fast Charge Control Register (0x03) Values
#define BQ25120A_FAST_CHARGE_RANGE_0_LOWER_mA       5.0f        /** 5mA */
#define BQ25120A_FAST_CHARGE_RANGE_0_UPPER_mA       35.0f       /** 35mA */
#define BQ25120A_FAST_CHARGE_RANGE_0_STEP_mA        1.0f        /** 1mA */
#define BQ25120A_FAST_CHARGE_RANGE_1_LOWER_mA       40.0f       /** 40mA */
#define BQ25120A_FAST_CHARGE_RANGE_1_UPPER_mA       300.0f      /** 300mA */
#define BQ25120A_FAST_CHARGE_RANGE_1_STEP_mA        10.0f       /** 10mA */


// Termination Pre-Charge Register (0x04) Values
#define BQ25120A_PRE_CHARGE_RANGE_0_LOWER_mA       0.5f        /** 0.5mA */
#define BQ25120A_PRE_CHARGE_RANGE_0_UPPER_mA       5.0f       /** 5mA */
#define BQ25120A_PRE_CHARGE_RANGE_0_STEP_mA        0.5f        /** 0.5mA */
#define BQ25120A_PRE_CHARGE_RANGE_1_LOWER_mA       6.0f       /** 6mA */
#define BQ25120A_PRE_CHARGE_RANGE_1_UPPER_mA       37.0f      /** 37mA */
#define BQ25120A_PRE_CHARGE_RANGE_1_STEP_mA        1.0f       /** 1mA */

// LDO Control Register (0x07) Values
#define BQ25120A_VBREG_DEFAULT_VOLTAGE              4.2f        /** 4.2V */
#define BQ25120A_BATT_VOLTAGE_MIN_V                 3.6f        /** 3.6V */
#define BQ25120A_BATT_VOLTAGE_MAX_V                 4.65f       /** 4.65V */
#define BQ25120A_BATT_VOLTAGE_STEP_V                0.01f       /** 0.01V */

// SYS VOUT Control Register (0x06) Values

/** A struct to represent the SYS_SEL and SYS_VOUT lookup table */
typedef struct
{
    float voltage;      /** The voltage in V */
    uint8_t SYS_SEL;    /** The SYS_SEL value */
    uint8_t SYS_VOUT;   /** The SYS_VOUT value */
} SYS_SEL_LookupTable_t;

/** Lookup table for SYS_SEL and SYS_VOUT values from Table 19 of the datasheet */
static const SYS_SEL_LookupTable_t SYS_SEL_LOOKUP_TABLE[] = {
    {1.1, 0x00, 0x00},      {1.2, 0x00, 0x01},      {1.25, 0x00, 0x02},     {1.3, 0x01, 0x00},
    {1.333, 0x00, 0x03},    {1.4, 0x01, 0x01},      {1.417, 0x00, 0x04},    {1.5, 0x00, 0x05},
    {1.5, 0x01, 0x02},      {1.5, 0x02, 0x00},      {1.583, 0x00, 0x06},    {1.583, 0x02, 0x01},
    {1.6, 0x01, 0x03},      {1.667, 0x00, 0x07},    {1.667, 0x02, 0x02},    {1.7, 0x01, 0x04},
    {1.75, 0x00, 0x08},     {1.75, 0x02, 0x03},     {1.8, 0x01, 0x05},      {1.8, 0x03, 0x00},
    {1.833, 0x00, 0x09},    {1.833, 0x02, 0x04},    {1.9, 0x01, 0x06},      {1.9, 0x03, 0x01},
    {1.917, 0x00, 0x0A},    {1.917, 0x02, 0x05},    {2.0, 0x00, 0x0B},      {2.0, 0x01, 0x07},
    {2.0, 0x02, 0x06},      {2.0, 0x03, 0x02},      {2.083, 0x00, 0x0C},    {2.083, 0x02, 0x07},
    {2.1, 0x01, 0x08},      {2.1, 0x03, 0x03},      {2.167, 0x00, 0x0D},    {2.167, 0x02, 0x08},
    {2.2, 0x01, 0x09},      {2.2, 0x03, 0x04},      {2.25, 0x00, 0x0E},     {2.25, 0x02, 0x09},
    {2.3, 0x01, 0x0A},      {2.3, 0x03, 0x05},      {2.333, 0x00, 0x0F},    {2.333, 0x02, 0x0A},
    {2.4, 0x01, 0x0B},      {2.4, 0x03, 0x06},      {2.417, 0x02, 0x0B},    {2.5, 0x01, 0x0C},
    {2.5, 0x02, 0x0C},      {2.5, 0x03, 0x07},      {2.583, 0x02, 0x0D},    {2.6, 0x01, 0x0D},
    {2.6, 0x03, 0x08},      {2.667, 0x02, 0x0E},    {2.7, 0x01, 0x0E},      {2.7, 0x03, 0x09}, 
    {2.75, 0x02, 0x0F},     {2.8, 0x01, 0x0F},      {2.8, 0x03, 0x0A},      {2.9, 0x03, 0x0B}, 
    {3.0, 0x03, 0x0C},      {3.1, 0x03, 0x0D},      {3.2, 0x03, 0x0E},      {3.3, 0x03, 0x0F}
}; 

/** The number of entries in the SYS_SEL_LOOKUP_TABLE */
static const unsigned int SYS_SEL_LOOKUP_TABLE_N_ENTRIES = sizeof(SYS_SEL_LOOKUP_TABLE) / sizeof(SYS_SEL_LOOKUP_TABLE[0]);

// LDO Control Register (0x07) Values
#define BQ25120A_LDO_MIN_V                          0.8f        /** 0.8V */
#define BQ25120A_LDO_MAX_V                          3.9f        /** 3.9V */
#define BQ25120A_LDO_STEP_V                         0.1f        /** 0.1V */


// INLIM & UVLO Voltage Register (0x09) Values

/** A struct to represent the SYS_SEL and SYS_VOUT lookup table */
typedef struct
{
    float voltage;      /** The voltage in V */
    uint8_t BUVLO;      /** The BUVLO value */
} BUVLO_LookupTable_t;

/** Lookup table for BUVLO values from Table 22 of the datasheet */
static const BUVLO_LookupTable_t BUVLO_LOOKUP_TABLE[] = {
    {2.2f, 0b111},
    {2.4f, 0b101},
    {2.6f, 0b100},
    {2.8f, 0b011},
    {3.0f, 0b010},
}; 

/** The number of entries in the BUVLO_LOOKUP_TABLE */
static const unsigned int BUVLO_LOOKUP_TABLE_N_ENTRIES = sizeof(BUVLO_LOOKUP_TABLE) / sizeof(BUVLO_LOOKUP_TABLE[0]);

#define BQ25120A_INLIM_MIN_mA                       50.0f       /** 50mA */
#define BQ25120A_INLIM_STEP_mA                      50.0f       /** 50mA */


#endif /* INC_BQ25120A_REG_H_ */
