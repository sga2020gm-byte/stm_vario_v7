
#include "main.h"
//#include "main.h"
//#include "main.c"

const uint32_t scale_factor [] = {524288UL, 1572864UL, 3670016UL, 7864320UL, 253952UL,
                                  516096UL, 1040384UL, 2088960UL};

extern I2C_HandleTypeDef hi2c1;

uint32_t p_scale = 253952UL, t_scale = 253952UL; //scale factor means from OSR for P & T
float last_meas_temp;

struct {
  int16_t c0, c1, c01, c11, c20, c21, c30;
  int32_t c00, c10;
} spl06_calib_data;

uint8_t spl06_ID_read (void){
	uint8_t reg_data;
	uint8_t reg_adres = SPL06_DEVICE_ID;
	//SPL06_DEVICE_ID
	HAL_I2C_Master_Transmit(&hi2c1, SPL06_ADDRESS, &reg_adres, 1,  100);
	HAL_I2C_Master_Receive(&hi2c1, SPL06_ADDRESS, &reg_data, 1, 100);
	return (reg_data);
}


uint8_t spl06_Init (void)
{
  uint8_t c_buff [18];
  uint8_t command;
  uint8_t status = 0;
  uint8_t count = 0;

  c_buff [0] = SPL06_SOFT_RESET;
  c_buff [1] = 0b1001;
  if (HAL_I2C_Master_Transmit(&hi2c1, SPL06_ADDRESS, c_buff, 2, 100) != HAL_OK) return TIMEOUT_ERR;
  HAL_Delay(10);

  while (status != 0b11000000) // ожидание готовности коэффициентов и завершения self-init
  {
    command = SPL06_MEAS_CFG;
    if (HAL_I2C_Master_Transmit(&hi2c1, SPL06_ADDRESS, &command, 1, 100) != HAL_OK) return TIMEOUT_ERR;
    if (HAL_I2C_Master_Receive(&hi2c1, SPL06_ADDRESS, c_buff, 1, 100) != HAL_OK) return TIMEOUT_ERR;

    status = c_buff[0] & 0b11000000; // COEF_RDY + SENSOR_RDY
    count++;
    if (count > 100) return TIMEOUT_ERR;
    HAL_Delay (10);
  }

  HAL_Delay (100);
  /*Получение калибровочных коэффициентов*/
  command = SPL06_COEF_C0;

  if (HAL_I2C_Master_Transmit(&hi2c1, SPL06_ADDRESS, &command, 1, 100) != HAL_OK) return TIMEOUT_ERR;
  if (HAL_I2C_Master_Receive(&hi2c1, SPL06_ADDRESS, c_buff, 18, 100) != HAL_OK) return TIMEOUT_ERR;

  spl06_calib_data.c0 = ((int16_t)c_buff[0] << 4) + ((c_buff[1] >> 4) & 0b00001111);
    if(spl06_calib_data.c0 & (1 << 11))
        {
        spl06_calib_data.c0 = spl06_calib_data.c0 | 0XF000; // Set left bits to one for 2's complement conversion of negative number
        }

  spl06_calib_data.c1 = ((int16_t)(c_buff[1] & 0b00001111) << 8) + c_buff[2];
    if(spl06_calib_data.c1 & (1 << 11))
        {
        spl06_calib_data.c1 = spl06_calib_data.c1 | 0XF000; // Set left bits to one for 2's complement conversion of negative number
        }

  spl06_calib_data.c00 = ((int32_t)c_buff[3] << 12) + ((int16_t)c_buff[4] << 4) + ((c_buff[5] >> 4) & 0b00001111);
    if(spl06_calib_data.c00 & (1 << 19))
        {
        spl06_calib_data.c00 = spl06_calib_data.c00 | 0XFFF00000; // Set left bits to one for 2's complement conversion of negative number
        }

  spl06_calib_data.c10 = ((int32_t)(c_buff[5] & 0b00001111) << 16) + ((int16_t)c_buff[6] << 8) + c_buff[7];
    if(spl06_calib_data.c10 & (1 << 19))
        {
        spl06_calib_data.c10 = spl06_calib_data.c10 | 0XFFF00000; // Set left bits to one for 2's complement conversion of negative number
        }

  spl06_calib_data.c01 = ((int16_t)c_buff[8] << 8) + c_buff[9];
  spl06_calib_data.c11 = ((int16_t)c_buff[10] << 8) + c_buff[11];
  spl06_calib_data.c20 = (c_buff[12] << 8) + (int16_t)c_buff[13];
  spl06_calib_data.c21 = ((int16_t)c_buff[14] << 8) + c_buff[15];
  spl06_calib_data.c30 = ((int16_t)c_buff[16] << 8) + c_buff[17];

  return OK;
}


uint8_t spl06_SetMode (uint8_t sensor_mode)
{
  uint8_t buff [2];
  uint8_t control_buff [1];
  buff [0] = SPL06_MEAS_CFG;
  buff [1] = sensor_mode;
  if (HAL_I2C_Master_Transmit(&hi2c1, SPL06_ADDRESS, buff, 2, 100) != HAL_OK) return TIMEOUT_ERR;
  //проверка
  if (HAL_I2C_Master_Transmit(&hi2c1, SPL06_ADDRESS, &buff[0], 1, 100) != HAL_OK) return TIMEOUT_ERR;
  if (HAL_I2C_Master_Receive(&hi2c1, SPL06_ADDRESS, control_buff, 1, 100) != HAL_OK) return TIMEOUT_ERR;
  if ((control_buff[0] & 0x07) != sensor_mode) return TIMEOUT_ERR;

    return OK;
}


uint8_t spl06_SetOSR (uint8_t press_osr, uint8_t temp_osr)
{
    uint8_t buff [2];
    uint8_t control_buff [2];
    uint8_t bit_shift = 0;

    p_scale = scale_factor [press_osr];
    t_scale = scale_factor [temp_osr];

    buff[0] = SPL06_PSR_CFG;
    if (HAL_I2C_Master_Transmit(&hi2c1, SPL06_ADDRESS, &buff[0], 1, 100) != HAL_OK) return TIMEOUT_ERR;
    if (HAL_I2C_Master_Receive(&hi2c1, SPL06_ADDRESS, &buff[1], 1, 100) != HAL_OK) return TIMEOUT_ERR;

    buff[1] = (0b0111 << 4) | press_osr; // PM_RATE=128Hz (максимальная частота по давлению)

    if (HAL_I2C_Master_Transmit(&hi2c1, SPL06_ADDRESS, buff, 2, 100) != HAL_OK) return TIMEOUT_ERR;
    if (HAL_I2C_Master_Receive(&hi2c1, SPL06_ADDRESS, control_buff, 2, 100) != HAL_OK) return TIMEOUT_ERR;

    buff[0] = SPL06_TMP_CFG;
    if (HAL_I2C_Master_Transmit(&hi2c1, SPL06_ADDRESS, &buff[0], 1, 100) != HAL_OK) return TIMEOUT_ERR;
    if (HAL_I2C_Master_Receive(&hi2c1, SPL06_ADDRESS, &buff[1], 1, 100) != HAL_OK) return TIMEOUT_ERR;

    buff[1] = ((0b0000 << 4) | temp_osr) | 1<<7; // TMP_RATE=1Hz + TMP_EXT=1 (температура вторична)

    if (HAL_I2C_Master_Transmit(&hi2c1, SPL06_ADDRESS, buff, 2, 100) != HAL_OK) return TIMEOUT_ERR;

    buff[0] = SPL06_CFG_REG;

    /*HAL_I2C_Master_Transmit(&hi2c1, SPL06_ADDRESS, &buff[0], 1, 100);;
    HAL_I2C_Master_Receive(&hi2c1, SPL06_ADDRESS, &bit_shift, 1, 100);*/

     if (press_osr >= SAMPLING_X16)  bit_shift = bit_shift | 1 << 2;
     else bit_shift = bit_shift & 0b11111011;
     if (temp_osr >= SAMPLING_X16)  bit_shift = bit_shift | 1 << 3;
     else bit_shift = bit_shift & 0b11110111;
     buff[1] = bit_shift;
     if (HAL_I2C_Master_Transmit(&hi2c1, SPL06_ADDRESS, buff, 2, 100) != HAL_OK) return TIMEOUT_ERR;

    return OK;
}
float spl06_ReadTemp (void)
{
   uint8_t buff [3];
   uint8_t command;
   int32_t T_raw;
   command = SPL06_TMP_B2;
   HAL_I2C_Master_Transmit(&hi2c1, SPL06_ADDRESS, &command, 1, 100);
   HAL_I2C_Master_Receive(&hi2c1, SPL06_ADDRESS, buff, 3, 100);

    T_raw = ((uint32_t)buff[0] << 16) + ((uint32_t)buff[1] << 8) + buff[2];
    if(T_raw & (1 << 23)){
        T_raw = T_raw | 0XFF000000; // Set left bits to one for 2's complement conversion of negative number
        }
    last_meas_temp = (float)T_raw / (float)t_scale;
    return last_meas_temp * (float)spl06_calib_data.c1 + (float)spl06_calib_data.c0 / 2.0f;
}

float spl06_ReadPressure (void)
{
   spl06_ReadTemp();
   uint8_t buff [3];
   uint8_t command;
   int32_t P_raw;
   float P_raw_sc;
   buff [0] = 0;
   buff [1] = 0;
   buff [2] = 0;
   command = SPL06_PSR_B2;
   HAL_I2C_Master_Transmit(&hi2c1, SPL06_ADDRESS, &command, 1, 100);
   //HAL_Delay(1);
   HAL_I2C_Master_Receive(&hi2c1, SPL06_ADDRESS, buff, 3, 100);

    P_raw = ((uint32_t)buff[0] << 16) + ((uint32_t)buff[1] << 8) + buff[2];
    if(P_raw & (1 << 23)){
        P_raw = P_raw | 0XFF000000; // Set left bits to one for 2's complement conversion of negative number
        }
    P_raw_sc = (float)P_raw / (float)p_scale;
    return (float)spl06_calib_data.c00 + P_raw_sc * ((float)spl06_calib_data.c10 + P_raw_sc * ((float)spl06_calib_data.c20 +
            P_raw_sc * (float)spl06_calib_data.c30)) + last_meas_temp * (float)spl06_calib_data.c01 + last_meas_temp * P_raw_sc * ((float)spl06_calib_data.c11 + P_raw_sc * (float)spl06_calib_data.c21);
}


float pascalToCentimeter(float pressurePa) {
    const float P0 = 101325.0f;
    const float SCALE = 44330.0f; // T0 / L

    if (pressurePa <= 0.0f)
        return 0.0f;

    // аппроксимация степенной формулы
    return 100 * (288.15f / 0.0065f) *  (1.0f - powf(pressurePa / P0, 0.190263f));
    //return 100.f * SCALE * (1.0f - expf(0.190263f * logf(pressurePa / P0)));
}
