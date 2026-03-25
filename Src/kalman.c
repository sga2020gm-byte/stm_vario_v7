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

/* This is a Kalman filter for vario/altimeter
	writen by Baranov
*/

#include "kalman.h"

#ifndef _Kalman_h
#define _Kalman_h


    void Filter_K(float *altitude, float *v_speed,float newz, float dt) {
        
        /* небольшая оптимизация */
        float dt2div2 = dt*dt/2.0f;
        float dt3div2 = dt2div2*dt;
        float dt4div4 = dt2div2*dt2div2;
        
        /* Шаг 1  */
        z += dt * vz;
        
        
        /* Шаг 2  Вычисляем ковариационную матрицу ошибок ( априорную )*/
       
        Kalman_P[0][0] += dt * (dt*Kalman_P[1][1] + Kalman_P[0][1] + Kalman_P[1][0]) + Kalman_Q_accel * dt4div4;
        Kalman_P[0][1] += dt * Kalman_P[1][1] + Kalman_Q_accel * dt3div2;
        Kalman_P[1][0] = Kalman_P[0][1];
        Kalman_P[1][1] += Kalman_Q_accel* dt * dt;
        
        /* Шаг 3 - Вектор ошибки измерения - в данном случае скаляр */
        Kalman_y = newz - z;

        /* Шаг 4 - Ковариационная матрица для вектора отклонения (вектора ошибки): - в данном случае скаляр */
        Kalman_S = Kalman_P[0][0] + Kalman_R_measure;
        /* Шаг 5  - вычисляем коэффициэты Калмана */
        Kalman_K[0] = Kalman_P[0][0] / Kalman_S;
        Kalman_K[1] = Kalman_P[1][0] / Kalman_S;
        
       
        /* Шаг 6 - коррекция положения  */
        z += Kalman_K[0] * Kalman_y;
        vz += Kalman_K[1] * Kalman_y;
        
        
        /* Step 7 Корректируем ковариационную матрицу ошибок */
        Kalman_P[0][0] -= Kalman_K[0] * Kalman_P[0][0];
        Kalman_P[0][1] -= Kalman_K[0] * Kalman_P[0][1];
        Kalman_P[1][0] = Kalman_P[0][1];
        Kalman_P[1][1] -= Kalman_K[1] * Kalman_P[0][1];
				
				*altitude = z;
				*v_speed = vz;
        
    };
    void setAlt(float newz) { z = newz; Kalman_P[0][0]=0.0f; }; // используется для установки начальной высоты
    
    /* Эти функции используются для настройки фильтра */
    void setQAccel(float newQ_accel) { Kalman_Q_accel = newQ_accel; };
    void setRmeasure(float newR_measure) { Kalman_R_measure = newR_measure; };

#endif
		//
