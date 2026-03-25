/**
 
   ----------------------------------------------------------------------
   	
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    any later version.
     
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
   -------
**/
#pragma once
/* Kalman filter variables */	 
				/* По умолчанию фильтр работает в следующих единицах измерения Выслта в см. вертикальная скорость в см/сек. ускорение в см/сек^2.
          Вектор состояния - пеерменные z,vz
        Установим дисперсию ускорения см/сек^2  - по сути среднее значение ускорения модели в врадрате - в моем случае процесс достаточно медленный. Примем эту величину
        равной 200см/сек за 3 секунды соответсвенно Q_accel=200/3*200/3=4444.4*/
 static float Kalman_Q_accel =2000.0f; // Дисперсия вертикального ускорения /* Эту величину можно подкорректировать методом setQAccel */

				/*Установим точность измерения высоты барметрического датчика. По datasheet-у у MS5607 - RMS (среднеквадратическое отклонение) в режиме сверх высокой точности равно 20 см
         соответсвенно дисперсия равна RMS^2 */
 static float Kalman_R_measure = 3000.0f; // Дисперсия датчика высоты /* Эту величину можно подкорректировать методом setRmeasure */
    
 static float z = 0; // Высота
 static float vz = 0; // Скорость изменения высоты
    
 static float Kalman_P[2][2] = {0.0f, 0.0f, 0.0f, 0.0f}; // Ковариационная матрица ошибок - 2x2 матрица
 static float Kalman_K[2]; // Коэффициенты Калмана - 2x1 матрица
 static float Kalman_y; // Ошибка измерения высот - 1x1 матрица
 static float Kalman_S; // Ковариационная матрица для вектора отклонения - 1x1 матрица	Вопрос?
 
 /* Заполним ковариационную матрицу ошибок
          Так-как мы не знаем тчного значения высоты на которой был включен прибор, и полагаем верикальную скорость равной 0, тогда
          P[0][0] - заполним достаточно большим числом Если высота изветна ( метод setAlt )тогда P[0][0]=0
        P[0][0] = 1000000000.0f; 
        P[0][1] = 0.0f;
        P[1][0] = P[0][1];   
				P[1][1] = 0.0f;
 Отсюда и далее в программе по формулам получается, что если в начальный момент P[0][1]=P[1][0]
                                то и по ходу программы ( модель такая ) везде это равенство сохраняется - и можно упростить ряд вычичлений
                                ,чтобы не грузить процессор лишней работой - или вообще для экономии память убрать P[0][1] или P[1][0] */
        
 
 void setAlt(float);
 void setQAccel(float);
 void setRmeasure(float);
 /* Функция фильтрации показаний высоты.
 Аргументы:
						указатель на переменную, в которую функция вернет высоту после фильтрации;
						указатель на переменную, в которую функция возвращает скороподъемность;
						новое показание высоты;
						время между предыдущим и текущим измерением в секундах
*/						
 
 void Filter_K(float *altitude, float *v_speed,float newz, float dt);
/*    END FILE      */ 
