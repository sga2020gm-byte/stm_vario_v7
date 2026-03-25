/*
 * general_func.h
 *
 *  Created on: Jul 2, 2025
 *      Author: GryaznevAA
 */

#ifndef GENERAL_FUNC_H_
#define GENERAL_FUNC_H_
#include "main.h"


#define EPD_COLOR_WHITE 0xFF
#define EPD_COLOR_BLACK 0x00


void display_full_upd();// постоянное обновление дисплея на протяжении определенного времени
void climb_calc(uint16_t *altitude,int16_t *climb_SNpS);//вычисление высоты и скороподъемности, фильтрация калманом
void display_inform(uint32_t altitude, int16_t climb_SNpS,uint8_t *text, uint8_t*image_bw);// отображение информации на дисплее
void Buzzer_Set(uint32_t frequency, uint8_t enable);
#endif /* GENERAL_FUNC_H_ */
void display_fields();
void Buzzer_inform_v2(int16_t climb);
void Check_Long_Press(uint8_t *image_bw);
void turn_of(uint8_t *image_bw);
