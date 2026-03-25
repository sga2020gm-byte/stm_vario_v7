
#define SENSOR_SPL06_001_H

#define spl06_delay       nrf_delay_ms      //Задержка для работы с ртос. При работе без ртос заменить nrf_delay_ms

#define SPL06_ADDRESS                  	  (0x77<<1) /**< Default I2C address from datasheet */
//#define SPL06_ADDRESS_ALT               0x77 /**< Alternate I2C address from datasheet */
//#include "main.h"

enum {
    SPL06_DEVICE_ID = 0x0D,
    SPL06_PSR_B2 = 0x00,
    SPL06_PSR_B1 = 0x01,
    SPL06_PSR_B0 = 0x02,
    SPL06_TMP_B2 = 0x03,
    SPL06_TMP_B1 = 0x04,
    SPL06_TMP_B0 = 0x05,
    SPL06_PSR_CFG = 0x06,
    SPL06_TMP_CFG = 0x07,
    SPL06_MEAS_CFG = 0x08,
    SPL06_CFG_REG = 0x09,
    SPL06_INT_STS = 0x0A,
    SPL06_FIFO_STS = 0x0B,
    SPL06_SOFT_RESET = 0x0C,
    SPL06_COEF_C0 = 0x10,
    SPL06_COEF_C0C1 = 0x11,
    SPL06_COEF_C1 = 0x12,
    SPL06_COEF_C00a = 0x13,
    SPL06_COEF_C00b = 0x14,
    SPL06_COEF_C00C10 = 0x15,
    SPL06_COEF_C10a = 0x16,
    SPL06_COEF_C10b = 0x17,
    SPL06_COEF_C01a = 0x18,
    SPL06_COEF_C01b = 0x19,
    SPL06_COEF_C11a = 0x1A,
    SPL06_COEF_C11b = 0x1B,
    SPL06_COEF_C20a = 0x1C,
    SPL06_COEF_C20b = 0x1D,
    SPL06_COEF_C21a = 0x1E,
    SPL06_COEF_C21b = 0x1F,
    SPL06_COEF_C30a = 0x20,
    SPL06_COEF_C30b = 0x21,
};


  /** Oversampling rate for the sensor. */
  enum sensor_sampling {
    SAMPLING_NONE = 0x00, /** No over-sampling. */
    SAMPLING_X2   = 0x01, /** 2x over-sampling. */
    SAMPLING_X4   = 0x02, /** 4x over-sampling. */
    SAMPLING_X8   = 0x03, /** 8x over-sampling. */
    SAMPLING_X16  = 0x04, /** 16x over-sampling. */
    SAMPLING_X32  = 0x05, /** 32x over-sampling. */
    SAMPLING_X64  = 0x06, /** 64x over-sampling. */
    SAMPLING_X128 = 0x07 /** 128x over-sampling. */
  };

  /** Operating mode for the sensor. */
  enum sensor_mode {
    MODE_STANDBY = 0x00, /** Standby mode. */
    MODE_FORCED_P = 0x01, /** Forced mode for Pressure. */
    MODE_FORCED_T = 0x02, /** Forced mode for Temperature. */
    MODE_BACKGND_P = 0x05, /** Background mode for Pressure. */
    MODE_BACKGND_T = 0x06, /** Background mode for Temperature. */
    MODE_BACKGND_BOTH = 0x07, /** Background mode for Both. */
    MODE_SOFT_RESET_CODE = 0x09 /** Software reset. */
  };

  /** measurement rate for sensor data. Applicable for background mode only*/
  enum sensor_rate {
    RATE_X1 = 0x00, /** 1 measurement per second. */
    RATE_X2 = 0x01, /** 2 measurement per second. */
    RATE_X4 = 0x02, /** 4 measurement per second. */
    RATE_X8 = 0x03, /** 8 measurement per second. */
    RATE_X16 = 0x04, /** 16 measurement per second. */
    RATE_X32 = 0x05, /** 32 measurement per second. */
    RATE_X64 = 0x06, /** 64 measurement per second. */
    RATE_X128 = 0x07 /** 128 measurement per second. */
  };

  enum spl06_errors {
    OK = 0,
    TIMEOUT_ERR = 4
  };
  uint8_t spl06_Init (void);

  /* Set sensor mode. As argumet needed value from enum 'sensor mode'

    Returns 'spl06_errors'*/
  uint8_t spl06_SetMode (uint8_t sensor_mode);

  /* Set oversampling for pressure and temp measurment
    For arguments needed 'sensor_sampling' */
  uint8_t spl06_SetOSR (uint8_t press_osr, uint8_t temp_osr);


  float spl06_ReadTemp (void);

  float spl06_ReadPressure (void);

  float pascalToCentimeter(float pressurePa);

