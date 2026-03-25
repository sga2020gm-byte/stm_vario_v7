/*
 * general_func.c
 *
 *  Created on: Jul 2, 2025
 *      Author: GryaznevAA
 */
#include "main.h"
uint32_t upd_time;
extern TIM_HandleTypeDef htim1;





void display_full_upd(){
	if ((upd_time+10000)<HAL_GetTick()||HAL_GetTick()<20000){ //переодическая отчистка дисплея
		//epd_paint_newimage(image_bw_ref, EPD_W, EPD_H, EPD_ROTATE_180, EPD_COLOR_WHITE);
		  //epd_init();
		  //epd_paint_clear(EPD_COLOR_WHITE);
		  //отрисовка полей отображения заново
		  epd_paint_drawLine(100,0,100,200,EPD_COLOR_BLACK,3);
		  epd_paint_drawLine(100,100,199,100,EPD_COLOR_BLACK,3);
		  //epd_displayBW(ref);
		  epd_update();
		  upd_time = HAL_GetTick();
		  epd_init_partial();
	}
}

void climb_calc(uint16_t *altitude,int16_t *climb_SNpS)
{
	static uint32_t now_time;
	static uint32_t last_time;
	static float dt;
	static float climb_float;
	static float alt_float;
	static uint16_t alt_int;
	static float pres_test;
	pres_test = spl06_ReadPressure();
	static int16_t climb_int;
	if (last_time+10<HAL_GetTick()){
		now_time = HAL_GetTick();
		dt =(now_time-last_time)/1000.;
		Filter_K(&alt_float, &climb_float, pascalToCentimeter(spl06_ReadPressure()),dt);
		last_time = HAL_GetTick();
		if (climb_float<30000){
			alt_int = alt_float;
			climb_int = climb_float;
			*altitude = alt_int;
			*climb_SNpS = climb_int;
		}
	}

}
void filter_init(){
	uint16_t cnt = 0;
	static float altitude;
	static float climb;
	while(cnt<10){
	Filter_K(&altitude, &climb, pascalToCentimeter(spl06_ReadPressure()),1);
	cnt++;
	}
}


void display_inform(uint32_t altitude, int16_t climb_SNpS, uint8_t *text, uint8_t* image_bw)
{
    static uint32_t old_time_view = 0;

    if ((HAL_GetTick() - 300) > old_time_view) {
        //epd_paint_clear(EPD_COLOR_WHITE);
        epd_paint_drawLine(100, 100, 100, 199, EPD_COLOR_BLACK, 3);
        epd_paint_drawLine(0, 100, 199, 100, EPD_COLOR_BLACK, 3);

        // --- Часть 1: climb_SNpS (+/-X.X) ---
        uint8_t climb_text[6]; // +/-X.X\0

        uint16_t absolute_value;
        uint8_t sign_char = '+';

        if (climb_SNpS < 0) {
            sign_char = '-';
            absolute_value = (uint16_t)(-climb_SNpS);
        } else {
            absolute_value = (uint16_t)climb_SNpS;
        }

        if (absolute_value > 99) {
            absolute_value = 99;
        }

        // Формируем строку
        climb_text[0] = sign_char;
        climb_text[1] = '0' + (absolute_value / 10);
        climb_text[2] = '.';
        climb_text[3] = '0' + (absolute_value % 10);
        climb_text[4] = '\0';

        epd_paint_showString(40, 20, climb_text, 50, EPD_COLOR_BLACK);

        // --- Часть 2: altitude ---
        // Используем общий буфер text для высоты
        uint32_t temp = altitude;
        uint8_t *ptr = text;

        // Сначала определим длину числа
        uint8_t digits = 0;
        uint32_t temp2 = altitude;
        do {
            digits++;
            temp2 /= 10;
        } while (temp2 > 0);

        // Заполняем строку с конца
        ptr += digits;
        *ptr = '\0'; // завершающий нуль

        do {
            ptr--;
            *ptr = '0' + (temp % 10);
            temp /= 10;
        } while (temp > 0);

        // Теперь text содержит строку с высотой
        epd_paint_showString(10, 140, text, 25, EPD_COLOR_BLACK);

        epd_displayBW_partial(image_bw);
        old_time_view = HAL_GetTick();
    }
}
/*void display_inform(uint32_t altitude, int16_t climb_SNpS, uint8_t *text, uint8_t* image_bw)
{
	static uint32_t old_time_view = 0;
	if ((HAL_GetTick()-300) > old_time_view){
    //epd_paint_clear(EPD_COLOR_WHITE);
    epd_paint_drawLine(100, 100, 100, 199, EPD_COLOR_BLACK, 3);
    epd_paint_drawLine(0, 100, 199, 100, EPD_COLOR_BLACK, 3);

    // Форматирование climb_SNpS с одной цифрой после точки
    uint16_t absolute_value = abs(climb_SNpS);
    if (absolute_value >99){
    	absolute_value = 99;
    }

    // Всегда выводим с одной цифрой после точки
    uint16_t whole_part = absolute_value / 10;
    uint16_t decimal_part = absolute_value % 10;
    sprintf((char*)text, "%c%d%c%d", (climb_SNpS < 0) ? '-' : '+', whole_part,'.', decimal_part);

    epd_paint_showString(40, 20, text, 50, EPD_COLOR_BLACK);

    sprintf((char*)text, "%d", altitude);
    epd_paint_showString(10,140, text, 25, EPD_COLOR_BLACK);
    //epd_displayBW_partial(image_bw);
    old_time_view = HAL_GetTick();
	}
}*/
void display_fields(uint8_t* image_bw)
{
	  epd_paint_clear(EPD_COLOR_WHITE);
	  epd_paint_drawRectangle(1, 1, 199, 199, EPD_COLOR_WHITE, 0);
	  epd_paint_drawLine(100,100,100,199,EPD_COLOR_BLACK,3);
	  epd_paint_drawLine(100,100,199,100,EPD_COLOR_BLACK,3);
	  epd_displayBW_partial(image_bw);
}

/**
 * @brief Управление пьезодинамиком на PA10 (TIM1_CH3)
 * @param frequency Частота сигнала в Гц
 * @param enable 1 — включить звук, 0 — выключить
 */
void Buzzer_Set(uint32_t frequency, uint8_t enable) {
    if (enable && frequency > 0) {
        uint32_t timer_clock = HAL_RCC_GetPCLK2Freq();  // TIM1 на APB2
        uint32_t prescaler = 0;
        uint32_t period = 0;

        // APB2 умножается на 2, если предделитель не 1
        if ((RCC->CFGR & RCC_CFGR_PPRE2) != RCC_CFGR_PPRE2_DIV1) {
            timer_clock *= 2;
        }

        // Подбор делителя и периода
        for (prescaler = 0; prescaler < 0xFFFF; prescaler++) {
            period = (timer_clock / (prescaler + 1)) / frequency;
            if (period <= 0xFFFF) break;
        }

        htim1.Instance->PSC = prescaler;
        htim1.Instance->ARR = period - 1;
        htim1.Instance->CCR3 = (period - 1) / 2;  // 50% скважность

        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    } else {
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    }
}
void Buzzer_inform(int16_t climb) {
    static uint32_t last_change_time = 0;
    static uint8_t is_beeping = 0;
    static uint32_t current_freq = 0;
    static uint16_t current_duration = 0;
    static uint16_t pause_duration = 0;

    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - last_change_time;

    // Если звук активен и время истекло - выключаем
    if (is_beeping && (elapsed >= current_duration)) {
        Buzzer_Set(0, 0);
        is_beeping = 0;
        last_change_time = now;
        pause_duration = current_duration / 2;  // Пауза = половина длительности
    }
    // Если звук неактивен и пришло время нового бипа - включаем (если climb в нужном диапазоне)
    else if (!is_beeping && (elapsed >= pause_duration)) {
        // Проверяем условия для звука
        if (climb > 1 || climb < -20) {
            // Рассчитываем параметры звука только для нужных climb
            if (climb > 1) {
                current_duration = (6000 - (100 * climb)) / 10;
                if (current_duration < 100) current_duration = 100;
                current_freq = 1000 + (100 * climb);
                if (current_freq > 3500) current_freq = 3500;
            }
            else if (climb < -20) {
                current_duration = 3000;
                current_freq = 800;
            }

            // Включаем звук
            Buzzer_Set(current_freq, 1);
            is_beeping = 1;
            last_change_time = now;
        }
        else {
            // Для climb от -2 до 1 просто сбрасываем таймер без звука
            last_change_time = now;
            pause_duration = 100;  // Короткая пауза перед следующей проверкой
        }
    }
}

void Buzzer_inform_v2(int16_t climb)
{
    static uint32_t last_time = 0;
    static uint8_t beeping = 0;

    uint32_t now = HAL_GetTick();

    /* ====== МЁРТВАЯ ЗОНА ====== */
    if (climb > -2 && climb < 2) {
        Buzzer_Set(0, 0);
        beeping = 0;
        return;
    }

    /* ====== НАБОР ВЫСОТЫ ====== */
    if (climb >= 2) {
        if (climb > 50) climb = 50;

        /* Частота: 1200 → 3200 Гц */
        uint32_t freq = 1200 + climb * 40;
        if (freq > 3200) freq = 3200;

        /* Длительность бипа: 120 → 40 мс */
        uint16_t beep_time = 140 - climb * 2;
        if (beep_time < 40) beep_time = 40;

        /* Пауза: 300 → 60 мс */
        uint16_t pause_time = 300 - climb * 4;
        if (pause_time < 60) pause_time = 60;

        if (!beeping && (now - last_time >= pause_time)) {
            Buzzer_Set(freq, 1);
            beeping = 1;
            last_time = now;
        }
        else if (beeping && (now - last_time >= beep_time)) {
            Buzzer_Set(0, 0);
            beeping = 0;
            last_time = now;
        }
        return;
    }

    /* ====== СНИЖЕНИЕ ====== */
    if (climb <= -10) {
        if (climb < -50) climb = -50;

        /* Низкий непрерывный тон */
        uint32_t freq = 700 + climb * 4;  // чем ниже — тем ниже тон
        if (freq < 300) freq = 300;

        Buzzer_Set(freq, 1);
        beeping = 1;
    }
}
void turn_of(uint8_t *image_bw)
{
	//epd_paint_selectimage(image_bw);
	epd_init_partial();
	uint8_t end_text[] = "TURN OF";

    epd_paint_clear(EPD_COLOR_WHITE);
    epd_paint_showString(10,83, end_text, 35, EPD_COLOR_BLACK);
	epd_displayBW_partial(image_bw);
    HAL_Delay(3000);

	epd_paint_drawRectangle(0, 0, 199, 199, EPD_COLOR_WHITE, 1);
	epd_displayBW_partial(image_bw);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
	HAL_Delay(500);
}


void Check_Long_Press(uint8_t*image_bw){
    static uint32_t press_time = 0;
    static uint8_t pressed = 0;

    if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == 1)
    {
        if(!pressed)
        {
            press_time = HAL_GetTick();

        }
        pressed = 1;

        if((HAL_GetTick() - press_time) >= 5000)
        {
        	turn_of(image_bw);
        }
    }
    else
    {
        pressed = 0;
        press_time = 0;
    }
}

