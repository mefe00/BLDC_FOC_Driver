#ifndef __HALL_H
#define __HALL_H

//  <<<------ MCU'nun kütüphanesini dahil etmek içindir. ------>>>
#include "stm32g4xx_hal.h"

//  <<<------------------------------------------------------------------------------->>>

//  <<<------ Kullanılan Değişkenler ------>>>

extern volatile float Electrical_Angle; // Motorun elektriksel pozisyonunu tutan ana değişken

//  <<<------------------------------------------------------------------------------->>>

//  <<<------ Fonksiyon Prototipleri ------>>>

void HALL_Init(TIM_HandleTypeDef *htim_hall); // Hall sensörlerini başlatma fonksiyonu (Fonksiyona kullanılan Timer adresi girilir)
float HALL_GetElectricalAngle(void); // Mevcut elektriksel rotor açısını döndüren fonksiyon
uint8_t HALL_GetCurrent_Sector(void); // Mevcut motor sektörünü döndüren fonksiyon
float HALL_GetSpeed_RPM(void); // Motorun elektriksel hızını RPM cinsinden döndüren fonksiyon


#endif // __HALL_H