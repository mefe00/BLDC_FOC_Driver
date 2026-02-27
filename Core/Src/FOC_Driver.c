// <<---------------------------------------------->>
// <<-------------Kütüphane Tanımlamaları---------->>
// <<---------------------------------------------->>

#include "FOC_Driver.h"
#include "stm32g4xx_ll_cordic.h" // LL kütüphanesini kullandığını varsayıyorum
#include "math.h"

// <<---------------------------------------------->>
// <<-------------Fonksiyon Tanımlamaları---------->>
// <<---------------------------------------------->>

void FOC_Driver_Init(FOC_Handle_t *pHandle){
    // Girişleri sıfırla
    pHandle->input.i_a_meas = 0.0f;
    pHandle->input.i_b_meas = 0.0f;
    pHandle->input.w_rad_s = 0.0f;
    pHandle->input.Electrical_Angle_rad = 0.0f;
    pHandle->input.T_mot_ref = 0.0f;
    pHandle->input.U_bat = 0.0f;
    
    // State'leri sıfırla
    pHandle->state.i_q_ref = 0.0f;
    pHandle->state.i_d_ref = 0.0f;
    pHandle->state.i_alpha = 0.0f;
    pHandle->state.i_beta = 0.0f;
    pHandle->state.i_d = 0.0f;
    pHandle->state.i_q = 0.0f;
    pHandle->state.u_d_decoupling = 0.0f;
    pHandle->state.u_q_decoupling = 0.0f;
    pHandle->state.d_q_max_voltage = 0.0f;
    pHandle->state.i_d_memory = 0.0f;
    pHandle->state.i_q_memory = 0.0f;
    pHandle->state.u_d = 0.0f;
    pHandle->state.u_q = 0.0f;
    pHandle->state.u_x = 0.0f;
    pHandle->state.u_y = 0.0f;
    pHandle->state.u_ref_a = 0.0f;
    pHandle->state.u_ref_b = 0.0f;
    pHandle->state.u_ref_c = 0.0f;
    
    // Çıkışları sıfırla
    pHandle->output.duty_a = 0.0f;
    pHandle->output.duty_b = 0.0f;
    pHandle->output.duty_c = 0.0f;
}

// ------------------------------------------------------------------------------

void FOC_Clark_Park_Transform(FOC_Handle_t *pHandle){
    float i_a = pHandle->input.i_a_meas;
    float i_b = pHandle->input.i_b_meas;
    float alpha_rad = pHandle->input.Electrical_Angle_rad;
    float sin_val, cos_val;

    // Clark Dönüşümü
    pHandle->state.i_alpha = i_a;
    pHandle->state.i_beta = (0.5773502f) * (i_a + 2.0f * i_b); 

    // Park Dönüşümü
    FOC_G4_Cos_Sin_Calculate(alpha_rad, &cos_val, &sin_val);

    pHandle->state.i_d =  (pHandle->state.i_alpha * cos_val) + (pHandle->state.i_beta * sin_val);
    pHandle->state.i_q = -(pHandle->state.i_alpha * sin_val) + (pHandle->state.i_beta * cos_val);
}   

// ------------------------------------------------------------------------------

void FOC_Torq_Reference_Transform(FOC_Handle_t *pHandle){
    float I_s_max = pHandle->config.I_s_max;
    float T_mot_ref = pHandle->input.T_mot_ref;
    float pole_pairs = (float)pHandle->config.pole_pairs;
    float flux_linkage = pHandle->config.flux_linkage;

    // Torktan akıma geçiş: Iq_ref = (2/3) * (T_ref / (PP * Flux))
    pHandle->state.i_q_ref = (T_mot_ref * 2.0f) / (3.0f * pole_pairs * flux_linkage);
    pHandle->state.i_d_ref = 0.0f; // Manyetik akı zayıflatma (Flux Weakening) yoksa 0

    // Akım Limitleme
    if(pHandle->state.i_q_ref > I_s_max){
        pHandle->state.i_q_ref = I_s_max;
    }
    else if(pHandle->state.i_q_ref < -I_s_max){
        pHandle->state.i_q_ref = -I_s_max;
    }
}

// ------------------------------------------------------------------------------

void FOC_Voltage_Decoupling(FOC_Handle_t *pHandle){
    float w_rad_s = pHandle->input.w_rad_s;
    float L_d = pHandle->config.L_d;
    float L_q = pHandle->config.L_q;
    float flux_linkage = pHandle->config.flux_linkage;
    float i_d = pHandle->state.i_d;
    float i_q = pHandle->state.i_q;

    // Cross-coupling terimlerini hesapla
    pHandle->state.u_d_decoupling = -w_rad_s * L_q * i_q;
    pHandle->state.u_q_decoupling =  w_rad_s * (L_d * i_d + flux_linkage);
}

// ------------------------------------------------------------------------------

void FOC_Max_Voltage(FOC_Handle_t *pHandle){
    // SVPWM kullanıldığı için DC bus voltajının %57.7'si (1/sqrt(3)) kullanılabilir lineer bölge
    // Modulation Index'e göre bu değişebilir ama genelde:
    pHandle->state.d_q_max_voltage = pHandle->input.U_bat * 0.57735f;
}

// ------------------------------------------------------------------------------

void FOC_Direct_Current_Control_d(FOC_Handle_t *pHandle){
    float Kp = pHandle->config.Kp_d;
    float Ki = pHandle->config.Ki_d;
    float Ts = pHandle->config.Ts;
    float max_volt = pHandle->state.d_q_max_voltage;
    
    float error = pHandle->state.i_d_ref - pHandle->state.i_d;
    float proportional = Kp * error;
    
    // Integral + Anti-Windup (Clamping)
    pHandle->state.i_d_memory += Ki * error * Ts;
    if (pHandle->state.i_d_memory > max_volt) pHandle->state.i_d_memory = max_volt;
    if (pHandle->state.i_d_memory < -max_volt) pHandle->state.i_d_memory = -max_volt;

    float u_d_out = proportional + pHandle->state.i_d_memory + pHandle->state.u_d_decoupling;

    // Çıkış Limitleme
    if(u_d_out > max_volt) u_d_out = max_volt;
    else if(u_d_out < -max_volt) u_d_out = -max_volt;

    pHandle->state.u_d = u_d_out;
}

// ------------------------------------------------------------------------------

void FOC_Direct_Current_Control_q(FOC_Handle_t *pHandle){
    float Kp = pHandle->config.Kp_q;
    float Ki = pHandle->config.Ki_q;
    float Ts = pHandle->config.Ts;
    
    // Q ekseni için kalan voltaj limitini hesapla
    float u_d = pHandle->state.u_d;
    float max_volt_abs = pHandle->state.d_q_max_voltage;
    float limit_sq = (max_volt_abs * max_volt_abs) - (u_d * u_d);
    float limit_volts = (limit_sq > 0.0f) ? sqrtf(limit_sq) : 0.0f;

    float error = pHandle->state.i_q_ref - pHandle->state.i_q;
    float proportional = Kp * error;

    // Integral + Anti-Windup
    pHandle->state.i_q_memory += Ki * error * Ts;
    if (pHandle->state.i_q_memory > limit_volts) pHandle->state.i_q_memory = limit_volts;
    if (pHandle->state.i_q_memory < -limit_volts) pHandle->state.i_q_memory = -limit_volts;

    float u_q_out = proportional + pHandle->state.i_q_memory + pHandle->state.u_q_decoupling;

    // Çıkış Limitleme
    if(u_q_out > limit_volts) u_q_out = limit_volts;
    else if(u_q_out < -limit_volts) u_q_out = -limit_volts;

    pHandle->state.u_q = u_q_out;
}

// ------------------------------------------------------------------------------

void FOC_Inverse_Clark_Park_Transform(FOC_Handle_t *pHandle){
    float u_d = pHandle->state.u_d;
    float u_q = pHandle->state.u_q;
    float alpha_rad = pHandle->input.Electrical_Angle_rad;
    float sin_val, cos_val;

    FOC_G4_Cos_Sin_Calculate(alpha_rad, &cos_val, &sin_val);

    // Inverse Park (d,q -> alpha, beta)
    pHandle->state.u_x = (u_d * cos_val) - (u_q * sin_val); // Alpha
    pHandle->state.u_y = (u_d * sin_val) + (u_q * cos_val); // Beta
}

// ------------------------------------------------------------------------------

void FOC_SVPWM_Calculation(FOC_Handle_t *pHandle){
    // Midpoint Clamp Yöntemi (Space Vector Generator)
    float U_alpha = pHandle->state.u_x;
    float U_beta = pHandle->state.u_y;
    float U_DC = pHandle->input.U_bat;

    if(U_DC < 1.0f) U_DC = 12.0f; // Sıfıra bölme koruması

    // 1. Inverse Clark ile 3 faz potansiyellerini (Va, Vb, Vc) bul
    float Va = U_alpha;
    float Vb = (-0.5f * U_alpha) + (0.8660254f * U_beta);
    float Vc = (-0.5f * U_alpha) - (0.8660254f * U_beta);

    // 2. Min ve Max faz voltajlarını bul
    float V_max = Va;
    float V_min = Va;

    if (Vb > V_max) V_max = Vb;
    if (Vc > V_max) V_max = Vc;
    if (Vb < V_min) V_min = Vb;
    if (Vc < V_min) V_min = Vc;

    // 3. Zero Sequence Offset (Midpoint) hesapla
    float V_offset = -0.5f * (V_max + V_min);

    // 4. Duty Cycle Hesapla (0.0 ile 1.0 arası)
    pHandle->output.duty_a = ((Va + V_offset) / U_DC) + 0.5f;
    pHandle->output.duty_b = ((Vb + V_offset) / U_DC) + 0.5f;
    pHandle->output.duty_c = ((Vc + V_offset) / U_DC) + 0.5f;

    // Saturation (0-1 arası sınırla)
    if(pHandle->output.duty_a > 1.0f) pHandle->output.duty_a = 1.0f; else if(pHandle->output.duty_a < 0.0f) pHandle->output.duty_a = 0.0f;
    if(pHandle->output.duty_b > 1.0f) pHandle->output.duty_b = 1.0f; else if(pHandle->output.duty_b < 0.0f) pHandle->output.duty_b = 0.0f;
    if(pHandle->output.duty_c > 1.0f) pHandle->output.duty_c = 1.0f; else if(pHandle->output.duty_c < 0.0f) pHandle->output.duty_c = 0.0f;
}

// ------------------------------------------------------------------------------

void FOC_G4_Cos_Sin_Calculate(float angle_rad, float *cos_value, float *sin_value){
    int32_t input_q31;
    int32_t cos_q31, sin_q31;

    // Açıyı -PI ile +PI arasına sarmala
    while (angle_rad > 3.141592741f) angle_rad -= 6.283185482f;
    while (angle_rad < -3.141592741f) angle_rad += 6.283185482f;

    // Q31 formatına dönüştür
    input_q31 = (int32_t)(angle_rad * CONST_SCALE);

    // CORDIC Donanımına Yaz
    CORDIC->WDATA = input_q31;

    // CORDIC Donanımından Oku (Cos ve Sin sırası config'e göre değişebilir, genelde böyledir)
    cos_q31 = (int32_t)(CORDIC->RDATA); 
    sin_q31 = (int32_t)(CORDIC->RDATA);
    
    // Float'a geri çevir
    *cos_value = (float)cos_q31 * CONST_UNSCALE;
    *sin_value = (float)sin_q31 * CONST_UNSCALE;
}

// ------------------------------------------------------------------------------

// ANA DÖNGÜ FONKSİYONU
// Bu fonksiyon timer interrupt içinde çağrılmalıdır.
void FOC_Current_Controller(FOC_Handle_t *pHandle){

    if(pHandle->config.current_ctrl_mode == false){
        // FOC Kapalıysa çıkışları sıfırla
        pHandle->output.duty_a = 0.0f;
        pHandle->output.duty_b = 0.0f;
        pHandle->output.duty_c = 0.0f;
        
        // Integralleri resetle ki açılınca zıplamasın
        pHandle->state.i_d_memory = 0.0f;
        pHandle->state.i_q_memory = 0.0f;
        return;
    }

    // 1. Ölçümleri al ve Dönüştür (Clarke & Park)
    FOC_Clark_Park_Transform(pHandle);

    // 2. İstenen torku akıma çevir
    FOC_Torq_Reference_Transform(pHandle);

    // 3. Voltaj Limitlerini ve Decoupling hesapla
    FOC_Max_Voltage(pHandle);
    FOC_Voltage_Decoupling(pHandle);

    // 4. PI Kontrolcüleri Çalıştır
    FOC_Direct_Current_Control_d(pHandle);
    FOC_Direct_Current_Control_q(pHandle);

    // 5. Ters Dönüşüm (Inverse Park) -> (u_d, u_q) to (u_alpha, u_beta)
    FOC_Inverse_Clark_Park_Transform(pHandle);

    // 6. PWM Duty Hesapla (SVPWM)
    FOC_SVPWM_Calculation(pHandle);
}



