#ifndef FOC_DRIVER_H_
#define FOC_DRIVER_H_

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "stm32g4xx.h" // CORDIC ve MCU register tanımları için gerekli

// <<---------------------------------------------->>
// <<----------- Değişken tanımlamaları ----------->>
// <<---------------------------------------------->>

#define CONST_SCALE 683565276.4f
#define CONST_UNSCALE 0.000000001463f

// FOC Algortimasının giriş yapıları
typedef struct{

//  << ---- Motor Parametreleri ---- >>
    uint8_t pole_pairs; // Motor kutup sayısı
    float R_phase;      // Faz direnci
    float L_d;          // d ekseni endüktansı
    float L_q;          // q ekseni endüktansı
    float flux_linkage; // Kalıcı mıknatıs akı halkalanması (Weber)

//  << ---- Sistem Limit Parametreleri ---- >>
    float voltage_limit;   // Maksimum voltaj limiti
    float current_limit;   // Maksimum akım limiti (I_s_max)
    float max_speed_rad_s; // Maksimum hız limiti (radyan/saniye)
    float I_s_max;         // Güvenlik için maksimum sistem akımı

//  << ---- PID Katsayıları ---- >>
    float Kp_d;
    float Ki_d;
    float Kp_q;
    float Ki_q;
    float Ts; // Örnekleme süresi (PID için sn cinsinden, örn: 0.0001)

    bool current_ctrl_mode;  // FOC algoritmasını aktif/deaktif etmek için

} FOC_Driver_Config_t;

// ------------------------------------------------------------------------------

typedef struct{
//  << ---- Ölçülen Giriş Değerleri ---- >>
    float i_a_meas;             // Ölçülen faz A akımı
    float i_b_meas;             // Ölçülen faz B akımı
    float w_rad_s;              // Motorun açısal hızı
    float Electrical_Angle_rad; // Elektriksel açı
    float T_mot_ref;            // Referans tork
    float U_bat;                // Batarya voltajı

} FOC_Driver_Input_t;

// ------------------------------------------------------------------------------

typedef struct{
//  << ---- Hesaplanan Değerler ---- >>
    float i_q_ref;
    float i_d_ref;

    float i_alpha;
    float i_beta;

    float i_q;
    float i_d;

    float u_d_decoupling;
    float u_q_decoupling;

    float d_q_max_voltage;

    float i_d_memory; // Integral birikimi D
    float i_q_memory; // Integral birikimi Q

    float u_d;
    float u_q;
 
    float u_x; // Alpha (Inverse Park sonrası)
    float u_y; // Beta  (Inverse Park sonrası)

    // Inverse Clark sonrası 3 faz voltajları (SVPWM öncesi ham voltajlar)
    float u_ref_a; 
    float u_ref_b;
    float u_ref_c;     

} FOC_Driver_State_t;

typedef struct{
// << ---- SVPWM bloğu sonuçları (0.0 - 1.0 arası) ---- >>
    float duty_a;
    float duty_b;
    float duty_c;  

} FOC_Driver_Output_t;

// ------------------------------------------------------------------------------

// FOC Ana Nesnesi
typedef struct{
    FOC_Driver_Config_t config;
    FOC_Driver_Input_t input;
    FOC_Driver_State_t state;
    FOC_Driver_Output_t output;
} FOC_Handle_t;

// <<---------------------------------------------->>
// <<------------- Fonksiyon Tanımlamaları -------->>
// <<---------------------------------------------->>

void FOC_Driver_Init(FOC_Handle_t *pHandle);
void FOC_Clark_Park_Transform(FOC_Handle_t *pHandle);
void FOC_Torq_Reference_Transform(FOC_Handle_t *pHandle);
void FOC_Current_Controller(FOC_Handle_t *pHandle); // Ana kontrol döngüsü
void FOC_Voltage_Decoupling(FOC_Handle_t *pHandle);
void FOC_Max_Voltage(FOC_Handle_t *pHandle);
void FOC_Direct_Current_Control_d(FOC_Handle_t *pHandle); 
void FOC_Direct_Current_Control_q(FOC_Handle_t *pHandle); 
void FOC_Inverse_Clark_Park_Transform(FOC_Handle_t *pHandle);
void FOC_SVPWM_Calculation(FOC_Handle_t *pHandle);
void FOC_G4_Cos_Sin_Calculate(float angle_rad, float *cos_value, float *sin_value);

#endif /* FOC_DRIVER_H_ */