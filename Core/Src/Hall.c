//  <<<------------------------------------------------------------------------------->>>
//  <<<------------------------------Driver Hakkında---------------------------------->>>
//  <<<------------------------------------------------------------------------------->>>

//  <<<-----------------------------Tanıtım ve Bilgilendirme-------------------------->>>
// Bu sürücü, hoverboard motorlarda olan BLDC motorlar için Hall sensörlerinden elektriksel açı bilgisi okumak için tasarlanmıştır.
// Sürücüde 3 tane Hall sensöründen gelen dijital sinyaller okunur ve bu sinyaller ile elektriksel açı değeri hesaplanır.
// Eğer daha hassas açı bilgisi isteniyorsa bu Hall sensörleri yerine manyetik encoder gibi daha yüksek çözünürlüklü (14-bit) encoderlar kullanılmalıdır.
// Motoru eğer FOC kontrol yöntemi ile sürmeyi planlıyorsanız ve robot kol gibi sistemlerde kullanacaksanız manyetik encoder kullanmadan açı bilgisini FOC algoritmasına girmek baş ağrıtabilir.
//  <<<------------------------------------------------------------------------------->>>

//  <<<-------------------------------------Yöntem------------------------------------>>>
// Üç Hall sensöründen gelen dijital sinyaller 3 bitlik bir sektör bilgisi oluşturur.
// Bu sektör bilgisi 1-6 arasında değişir ve her sektör 60 derecelik bir elektriksel açı aralığını temsil eder.
// Sektör bilgisi kullanılarak temel elektriksel açı değeri belirlenir.
// Ayrıca her sektör geçişinde geçen süre ölçülerek sektör içi açı doğrusal olarak hesaplanır.
// Toplam elektriksel açı, sektör taban açısı ile sektör içi açının toplamı olarak elde edilir.
// | Ha | Hb | Hc | Sektör | Elektriksel Açı Aralığı | Açı ortası FOC için |
// |----|----|----|--------|-------------------------|---------------------|
// | 0  | 0  | 1  |   1    |      330 - 30           |        0            |
// | 0  | 1  | 0  |   2    |      90 - 150           |        120          |
// | 0  | 1  | 1  |   3    |      30 - 90            |        60           |
// | 1  | 0  | 0  |   4    |      210 - 270          |        240          |
// | 1  | 0  | 1  |   5    |      270 - 330          |        300          |
// | 1  | 1  | 0  |   6    |      150 - 210        s  |        180          |  

// NOT: Tipik bir 3 fazlı BLDC motorun normal çalışmasında tüm hall sensörlerinin aynı anda 0 veya 1 verdiği görülmez.
//  <<<------------------------------------------------------------------------------->>>

//  <<<---------------------------------Kullanımı------------------------------------->>>

// 1. HALL_Init fonksiyonu ile Hall sensörleri için gereken başlangıç ayarlarını yapın.
//    Bu fonksiyona Hall sensörleri için kullanılan Timer'ın adresini girin.
// 4. HALL_GetElectricalAngle fonksiyonu ile mevcut elektriksel açıyı derece cinsinden okuyabilirsiniz.
//    float angle = HALL_GetElectricalAngle();
// 5. HALL_GetCurrentSector fonksiyonu ile mevcut sektörü (1-6) okuyabilirsiniz.

// Yapılması gereken MX Konfigürasyonlar (STM32G431CBU6):
// Timer olarak TIM4 veya TIM2 kullanılabilir.
// Combined Channels olarak XOR ON/ Hall sensör modunda ayarlanmalıdır.
// Prescaler: 169
// Counter Mode: Up
// Dithering Mode: Disabled
// Counter Period: 65535
// Internal Clock: No Division
// auto Reload Preload: Disabled
// Trigger Output Selection TRGO: Output Compare (OC1REF)
// Prescaler Division Ratio: No Division
// Polarity: Rising Edge
// Input Fılter: 10-15 Hz
// Commutation Delay: 0

//  <<<------------------------------------------------------------------------------->>>


//  <<<------------------------------------------------------------------------------->>>
//  <<<------------------------------------------------------------------------------->>>
//  <<<------------------------------------------------------------------------------->>>

#include "Hall.h"
#include "stm32g4xx_hal.h"
//  <<<------------------------------------------------------------------------------->>>
//  <<<------ Özel Değişkenler ------>>>
//  <<<------------------------------------------------------------------------------->>>


static TIM_HandleTypeDef *HALL_htim; // Hall sensörleri için kullanılan Timer'ın adresi

volatile uint8_t sector = 0; // En son okunan 60 derecelik sektör (1-6)
volatile uint32_t sector_previous_time = 0; // Önceki sektör geçiş süresi (Sektörün içinden detaylı açı hesabı için)
volatile float Electrical_Angle = 0.0f; 
volatile int8_t rotation_direction = 1; // Motorun dönüş yönü (1: Saat yönü, -1: Saat yönünün tersi)
volatile uint32_t last_capture_time = 0; // Motorun durma kontrolü için
volatile uint32_t sector_duration = 0; // 60 derecelik geçiş süresi (CCR1'den gelir)
volatile int8_t direction = 1; // Motorun yönü (1: Saat yönü, -1: Saat yönünün tersi)

// Bunu kendi motor kutup sayına göre ayarlamalısın! (Hoverboard genelde 15 çift kutuptur)
#define MOTOR_POLE_PAIRS 15

const uint8_t HALL_SECTOR_MAP[8]={
         //  Açı,  Hall_a, Hall_b, Hall_c, Sektör
    0,   // 000 (Geçersiz---> Tipik 3 fazlı bir BLDC motorun normal çalışmasında tüm hall sensörlerinin aynı anda 0 verdiği görülmez),
    330, // 001 (Sektör 1)
    90,  // 010 (Sektör 2)
    30,  // 011 (Sektör 3) 
    210, // 100 (Sektör 4)
    270, // 101 (Sektör 5)
    150, // 110 (Sektör 6)
    0    // 111 (Geçersiz---> Tipik 3 fazlı bir BLDC motorun normal çalışmasında tüm hall sensörlerinin aynı anda 1 verdiği görülmez).
};

//  <<<------------------------------------------------------------------------------->>>
//  <<<------------------------------------------------------------------------------->>>
//  <<<------------------------------------------------------------------------------->>>




//  <<<------------------------------------------------------------------------------->>>
//  <<<------ Fonksiyon Uygulamaları ------>>>
//  <<<------------------------------------------------------------------------------->>>

// Hall sensörleri için gereken başlangıç ayarlarını yapar
void HALL_Init(TIM_HandleTypeDef *htim_hall) {
    
    HALL_htim = htim_hall; // Kullanıcının girdiği timer adresini kaydettik.
    HAL_TIMEx_HallSensor_Start_IT(HALL_htim);// Kullanıcı için Timer'ı başlattık.

    // Sektör aslında 3 bitlik bir değerdi ve bu üç biti hall sensörlerinden okunan digital girişlerdi
    // Eğer sektörü bilmek istiyorsak hall sensörlerinin durumunu okumamız gerekiyor.
    uint8_t Hall_A = HAL_GPIO_ReadPin(HALL_A_GPIO_Port, HALL_A_Pin);
    uint8_t Hall_B = HAL_GPIO_ReadPin(HALL_B_GPIO_Port, HALL_B_Pin);
    uint8_t Hall_C = HAL_GPIO_ReadPin(HALL_C_GPIO_Port, HALL_C_Pin);

    // Okunan değerleri birleştirip sektör belirleyeceğiz.
    sector = (Hall_A << 2) | (Hall_B << 1) | (Hall_C);

    if(sector > 7) sector = 0; // Güvenlik amaçlı geçersiz sektör kontrolü
    Electrical_Angle = (float)HALL_SECTOR_MAP[sector]; // İlk açıyı belirledik.

    //İlk geçiş süresini sıfırlıyoruz çünkü dönmeyen motor için sektör zamanı sonsuzdur.
    //sector_previous_time = 0;
}

//  <<<------------------------------------------------------------------------------->>>


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
    if(htim == HALL_htim){
        // Her 60 derecelik geçişte bu fonksiyon çağrılır.
        sector_duration = HAL_TIM_ReadCapturedValue(HALL_htim, TIM_CHANNEL_1);
        
        // 2. Sadece hangi sektörde olduğumuzu anlamak için GPIO oku
        // (Hız hesabına gerek yok, onu donanım yaptı)
        uint8_t Hall_A = HAL_GPIO_ReadPin(HALL_A_GPIO_Port, HALL_A_Pin);
        uint8_t Hall_B = HAL_GPIO_ReadPin(HALL_B_GPIO_Port, HALL_B_Pin);
        uint8_t Hall_C = HAL_GPIO_ReadPin(HALL_C_GPIO_Port, HALL_C_Pin);

        uint8_t new_sector = (Hall_A << 2) | (Hall_B << 1) | (Hall_C);

        if(new_sector >= 1 && new_sector <= 6) {
            // Yön tespiti için eski ve yeni sektörü karşılaştırabilirsin
            // Şimdilik basit tutuyoruz
            sector = new_sector;
            Electrical_Angle = (float)HALL_SECTOR_MAP[sector];
        }
        
        // Son geçiş zamanını güncelliyoruz.
        last_capture_time = HAL_GetTick();
    }
}

//  <<<------------------------------------------------------------------------------->>>

float HALL_GetElectricalAngle(void){
    
    // Sektör içi açı hesabı (Doğrusal İnterpolasyon)
    float Sector_Angle_Inter = 0.0f;
    
    if((HAL_GetTick() - last_capture_time) > 100){
    
        sector_duration = 0; // Motor durduysa süre sonsuzdur.
        return (float)HALL_SECTOR_MAP[sector]; // Sadece sektör taban açısını döneriz.
    }
    
    if(sector_duration > 0){
        // Geçen süreyi kullanarak sektör içi açıyı hesapla
        // Sektör içi açı = (Geçen süre / Toplam sektör süresi) * 60 derece
        uint32_t current_time = __HAL_TIM_GET_COUNTER(HALL_htim);

        Sector_Angle_Inter = ((float)current_time / (float)sector_duration) * 60.0f;

        if(Sector_Angle_Inter > 60.0f){
            Sector_Angle_Inter = 60.0f; // Güvenlik kontrolü
        }
    }
        // Açı(toplam) = Açı(sektör taban) + Açı(içi)
        float Total_Angle = (float)HALL_SECTOR_MAP[sector] + Sector_Angle_Inter;

        if(Total_Angle >= 360.0f){  
            Total_Angle -= 360.0f;
        }

        return Total_Angle;  
}   

//  <<<------------------------------------------------------------------------------->>>


float HALL_GetSpeed_RPM(void){

    if (sector_duration || (HAL_GetTick() - last_capture_time) > 100) {
        return 0.0f; // Motor durduysa hız sıfırdır.
    }

    //RPM Formulü: (Timer_Freq * 60) / (Duration * 6 * Pole_Pairs)
    float timer_freq = 1000000.0f; 
    float rpm = (timer_freq * 60.0f) / ((float)sector_duration * 6.0f * (float)MOTOR_POLE_PAIRS);
    
    return rpm;
}

//  <<<------------------------------------------------------------------------------->>>

// Mevcut elektriksel sektörü döner (1-6)
uint8_t HALL_GetCurrentSector(void){
    return sector;
}


//  <<<------------------------------------------------------------------------------->>>
//  <<<------------------------------------------------------------------------------->>>
//  <<<------------------------------------------------------------------------------->>>