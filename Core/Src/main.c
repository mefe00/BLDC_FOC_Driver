/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "cordic.h"
#include "fdcan.h"
#include "fmac.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FOC_Driver.h"
#include "Hall.h"
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void FDCAN_Send_Telemetry(void);
void Process_Debug_Command(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/

// =======================================================================================================================
// =======================================================================================================================
// =======================================================================================================================

/* USER CODE BEGIN 0 */
// FOC kütüphanemizde oluşturduğumuz handle yapısı ile bir tane nesne tanımlıyoruz.
FOC_Handle_t motor1;

// Gerçek akım değerlerine çevirmek için offset tanımlamalarımızı yaptık.
// İdealde bu değer 3.3V / 2 = 1.65V olması gerekir ama donanımdan dolayı illaki sapmalar olacaktır.
float offset_ia = 0.0f;
float offset_ib = 0.0f;

// Kalibrasyon için kullanılacak bayrak ve sayaçlar
bool is_calibrated = false;
uint32_t calib_sum_a = 0;
uint32_t calib_sum_b = 0;
uint16_t calib_count = 0;
#define CALIBRATION_SAMPLES 1000 // 20kHz'de 1000 örnek = 50 milisaniye sürer

// CAN Bus haberleşmesinde tek paket içinde 8 Byte veri taşınabilir.
// Ana bilgisayar (Jetson, Raspberry Pi veya herhangi bir gömülü sistem) hedef hız ve hedef tork gönderecek.
// Bu veriler float veri tipinde gelebilir. Float veri paketi ise 4 byte yer kaplar.
// Biz bu 4 byte'lık veri paketini CAN paketlerine parçalayacağız.
// Bunun içinde Union kullanacağız. 
// Union hafızadaki aynı adresi farklı veri tipleri ile okumamı sağlar.
// Burada kullanılmasının sebebi: Normalde struct normal tanımlanırsa içindeki her değişken RAM'de kendine ait adreslere yerleşir.
// Fakat union tanımladığında değişkenlerin hepsi aynı adrese tanımlanır.
// Avantajı ise şudur: Jetson bize float veri gönderdiğinde (içinde her şeyin olduğu veri)....
// .... bu veriye biz converter.byte[0] şeklinde bakarsak o float verinin makine dilindeki parçalanmış ilk byte'nı doğrudan okuyabilirsin.
// Bunu şu şekilde de anlatmak isterim:
// Normalde biz direkt integer yani tam sayı alıyor olsak sıkıntı yok. Bit-Shifting yaparak byte'ları ayırabilirdik.
// Fakat C dilinde ondalıklar sayılarda bit-shifting yapmak yasaktır.
// Unıon kullandığımızda CPU'ya hiç matematiksel işlem yaptırmadan 4 byte'lık veriyi direkt CAN veri dizisine kopyalayabiliriz.

// Jetsondan gelecek örnek veri yapısı ( Byte 0: Kontrol için (0: Dur, 1: Tork Modu, 2: Hız Modu)
// Byte 1-4 : Gönderilen değer
typedef union {
    float float_val;
    uint8_t bytes[4];
} FloatConverter;

// Debug ve PID parametre değiştirmek için seri port veri tanımlamaları.
#define RX_BUFFER_SIZE 64
uint8_t rx_byte; // Anlık okunan tek karakter
char rx_buffer[RX_BUFFER_SIZE]; // Karakterlerin birleştiği kelime
uint8_t rx_index = 0;
bool command_ready = false;
/* USER CODE END 0 */

// =======================================================================================================================
// =======================================================================================================================
// =======================================================================================================================


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  
  // CAN verilerini ana bilgisayara loglarken zaman damgası eklemek için bir kronometre (timer) tanımlayalım.
  uint32_t last_telemetry_time = 0; // Kronometremizin başlangıç noktası

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CORDIC_Init();
  MX_FDCAN1_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_FMAC_Init();
  MX_TIM3_Init();

// =======================================================================================================================
// =======================================================================================================================
// =======================================================================================================================

  /* USER CODE BEGIN 2 */
  
  // Bu kısımda tamamen çevre birimlerini ayağa kaldırılır ve FOC ayarları ve ADC kalibrasyonu yapılır.
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET); // Buzzer ON
  HAL_Delay(100);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET); // Buzzer OFF
  // Hall sensörü TIMER'ının başlatılması
  HALL_Init(&htim3);
  // ADC çalıştırılmadan önce kalibre edilmesi gerekir.
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

  // FOC sürücüsünün parametrelerinin tanımlanması
  FOC_Driver_Init(&motor1);
  // Motorun parametreleri ( Bu değerler datasheet'ten alınır ve motorun gerçek değerlerine göre güncellenir.)
  motor1.config.pole_pairs = 15; // Motorun kutup sayısı
  motor1.config.R_phase = 0.5f; // Faz direnci (Ohm cinsinden)
  motor1.config.L_d = 0.0003f; // d ekseni endüktansı (Henry cinsinden)
  motor1.config.L_q = 0.0003f; // q ekseni endüktansı (Henry cinsinden)
  motor1.config.flux_linkage = 0.015f; // Kalıcı mıknatıs akı halkalanması (Weber cinsinden)
  // Sistem limit parametreleri
  motor1.config.voltage_limit = 36.0f; // Maksimum voltaj limiti
  motor1.config.current_limit = 15.0f; // Maksimum akım limiti (Bu değeri ADC'nin ölçebilceği max akım değerinden biraz daha az şekilde alacağız. MAX akım değeri ise şu formül ile hesaplanır Imax= ((V_max-V_offset)/(R_shontxGain)) ).
  motor1.config.I_s_max = 15.0f; // Güvenlik için maksimum sistem akımı
  motor1.config.Ts = 0.00005f; // Örnekleme süresi (20kHz olduğu için 1/f yaparak buluyoruz).
  motor1.config.current_ctrl_mode = true; // FOC algoritmasını aktif hale getiriyoruz.
  // PID katsayılarını gir. (İlk değerler olduğu için güvenlik amaçlı düşük tutulmuştur.)
  motor1.config.Kp_d = 0.1f;
  motor1.config.Ki_d = 0.01f;
  motor1.config.Kp_q = 0.1f;
  motor1.config.Ki_q = 0.01f;
  
  // Burada donanımın başlatılması için gereken ayarlar yapılır.
  // Sıralaması önemlidir çünkü ADC başlatıldıkta sonra tetiklenmesi gerekir.
  HAL_ADCEx_InjectedStart_IT(&hadc1);
  // Daha sonrasında TIM! PWM kanalları başlatılır.
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); 
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  // NOT: HAL_TIM_PWM_Start() fonksiyonu normal kanallar için, HAL_TIMEx_PWMN_Start() ise tamamlayıcı kanallar için kullanılır.

  // CAN bağlantısı başarılı bir şekilde kuruldumu kontrol amaçlı ayarın başında açıyorum. Sonunda kapatacağım
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); 

  // 1. Gelen Mesajlar İçin Filtre Ayarı (Sadece 0x11 ID'sini kabul et)
  FDCAN_FilterTypeDef sFilterConfig;
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // Gelenler FIFO0'a düşsün
  sFilterConfig.FilterID1 = 0x11; // Kabul edilecek ID (Bu kartın ID'si)
  sFilterConfig.FilterID2 = 0x7FF; // Maske (Tüm bitlerin tam eşleşmesini istiyoruz)

  HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);

  // 2. Global Filtre Ayarı: Filtreden geçemeyen (diğer motorların) mesajlarını Reddet (Reject)
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

  // 3. FDCAN Modülünü Başlat
  HAL_FDCAN_Start(&hfdcan1);

  // 4. Yeni mesaj geldiğinde Bize Haber Ver (Interrupt'ı aktif et)
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);  


  // Seri port üzerinden başlatılan kesme ile komut alma işlemi için UART kesmesini aktif ediyoruz.
  HAL_UART_Receive_IT(&huart3, &rx_byte, 1); 
  /* USER CODE END 2 */

// =======================================================================================================================
// =======================================================================================================================
// =======================================================================================================================


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    
    // 1. Yeni gelen debug komutu var mı diye sürekli kontrol et
    Process_Debug_Command();

    // Sistemin o anki saatini al
    uint32_t current_time = HAL_GetTick();

    // Son gönderimden bu yana 20 milisaniye geçti mi?
    if (current_time - last_telemetry_time >= 20) 
    {
        last_telemetry_time = current_time; // Kronometreyi sıfırla
        
        // --- A. FDCAN Üzerinden Jetson'a Telemetri Gönder ---
        FDCAN_Send_Telemetry(); 

        // --- B. Web Arayüzü İçin UART Üzerinden Veri Bas ---
        char tx_buffer[100];
        int len = snprintf(tx_buffer, sizeof(tx_buffer), ">V:%.1f,I:%.2f,W:%.1f,T:%.2f\r\n", 
                           motor1.input.U_bat, 
                           motor1.state.i_q, 
                           motor1.input.w_rad_s * 9.549f, // RPM
                           motor1.input.T_mot_ref);
                           
        HAL_UART_Transmit_IT(&huart3, (uint8_t*)tx_buffer, len);
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

// =======================================================================================================================
// =======================================================================================================================
// =======================================================================================================================

/* USER CODE BEGIN 4 */

// Bu kısımda artık ADC'den gelen veriler okunacak ama önemli bir detay var:
// Sadece ADC değerleri okuması değil biz CubeMX ayarları yaparken ve motorun daha sessiz dönmesini sağlamak için....
// ADC'nin tetiklenme kaynağını TIM1'in güncelleme olayına (Update Event) bağlamıştık. Yani her TIM1'in güncelleme olayında ADC tetiklenecek ve yeni bir veri seti okuyacak.
// Bu yüzden ADC'nin okuma işlemi kesme (Interrupt) ile yapılır. Yani ADC yeni bir veri okuduğunda bir kesme oluşturur ve biz de bu kesme içerisinde ADC verilerini okuyarak FOC algoritmamıza girdi olarak veririz. Bu sayede her zaman güncel verilere sahip oluruz ve motorumuzun performansı artar.
// ÖZET sıralama:
// 1. TIM1 sayıcısı 0 değerine iner.
// 2. TIM1 donanımsal olarak ADC'yi tetikler.
// 3. ADC hemen FAZA ve FAZB akımlarını okur ve JDR1 ve JDR2 register'larına yazar.
// 4. Akımlar okunduktan sonra açı değeri ile referans tork FOC'ye girer ve FOC algoritması çalışır.
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        // 1. ADC Raw Değerlerini Donanım Register'ından Şimşek Hızıyla Oku
        uint32_t raw_ia = hadc->Instance->JDR1; // Faz A
        uint32_t raw_ib = hadc->Instance->JDR2; // Faz B

        // <<--- KALİBRASYON AŞAMASI --->>
        if (!is_calibrated) 
        {
            calib_sum_a += raw_ia;
            calib_sum_b += raw_ib;
            calib_count++;
            
            // LED1 ile kalibrasyon durumunu kontrol edeceğiz. Eğer yanıp sönüyorsa kalibrasyon devam ediyor demektir. 
            if (calib_count % 200 == 0) {
                HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14); // LED1 Toggle
            }

            if (calib_count >= CALIBRATION_SAMPLES)
            {
                // 1000 örneğin ortalamasını al ve Voltaja çevir
                float avg_raw_a = (float)calib_sum_a / (float)CALIBRATION_SAMPLES;
                float avg_raw_b = (float)calib_sum_b / (float)CALIBRATION_SAMPLES;

                offset_ia = (avg_raw_a / 4095.0f) * 3.3f;
                offset_ib = (avg_raw_b / 4095.0f) * 3.3f;

                is_calibrated = true; // Kalibrasyon bitti, FOC başlayabilir!

                // Kalibrasyon bittiğide LED1'i sürekli yakıyoruzki bittiğini anlayalım.
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
            }
            
            // Kalibrasyon bitene kadar motoru tamamen serbest (0 Duty) bırakıyoruz
            TIM1->CCR1 = 0;
            TIM1->CCR2 = 0;
            TIM1->CCR3 = 0;
            return; // Kesmeden çık, FOC'a girme
        }

        // 2. Ham değerleri Voltaja, oradan da Amper'e (A) çevir
        float v_ia = ((float)raw_ia / 4095.0f) * 3.3f;
        float v_ib = ((float)raw_ib / 4095.0f) * 3.3f;
        
        motor1.input.i_a_meas = (v_ia - offset_ia) / 0.1f; 
        motor1.input.i_b_meas = (v_ib - offset_ib) / 0.1f;

        // 3. Batarya Voltajını Oku (Regular kanaldan okuduğunu varsayıyoruz)
        // motor1.input.U_bat = ... (Batarya okuma fonksiyonundan gelecek)
        motor1.input.U_bat = 36.0f; // Şimdilik sabit tutalım

        // 4. Hall Sensöründen Açı ve Hız Bilgilerini Al
        // Not: Dereceyi Radyana çeviren fonksiyonu (HALL_GetElectricalAngle_Rad) kullandık!
        motor1.input.Electrical_Angle_rad = HALL_GetElectricalAngle_Rad();
        motor1.input.w_rad_s = HALL_GetSpeed_RPM() * 0.104719755f; // RPM to Rad/s dönüşümü

        // 5. Jetson'dan gelen hedef torku ayarla (Şimdilik test için 0.5 Nm diyelim)
        motor1.input.T_mot_ref = 0.5f; 

        // 6. EFSANE FONKSİYONU ÇAĞIR (FOC Matematiği Çalışsın)
        FOC_Current_Controller(&motor1);
        
        // Failsafe durumları için (akım limiti aşıldı mı?)
       if (motor1.input.i_a_meas > motor1.config.I_s_max || motor1.input.i_a_meas < -motor1.config.I_s_max ||
        motor1.input.i_b_meas > motor1.config.I_s_max || motor1.input.i_b_meas < -motor1.config.I_s_max)
       {
        // Sisteme acil durdurma ver!
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); // LED3 (Hata) ON
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // Buzzer Alarm ON
        motor1.config.current_ctrl_mode = false; // FOC'u durdur
       }

        // 7. Hesaplanan Duty Cycle'ları (0.0 - 1.0 arası) Donanıma (Timer'a) Yaz
        // Direkt CCR register'larına yazmalar yaptık. Daha hızlı olması için döngünün.
        // Hatırla: ARR değerimizi 4250 olarak hesaplamıştık. (20kHz olarak seçtiğimiz için)
        TIM1->CCR1 = (uint32_t)(motor1.output.duty_a * 4250.0f); // Faz A
        TIM1->CCR2 = (uint32_t)(motor1.output.duty_b * 4250.0f); // Faz B
        TIM1->CCR3 = (uint32_t)(motor1.output.duty_c * 4250.0f); // Faz C
    }
}

// NOT: Motorları bu şekilde sürmemizin asıl sebebi ve açıklaması MOSFET'ler en sessiz olduğu zaman....
// En gürültüsüz olduğu zamandır ve bu zamanda akım okuması yapıp MOSFET'leri sürmek en prüzsüz ve hassas kontrolü sağlamanın yoludur.

// =======================================================================================================================

// CAN HATTI RX
// Bu fonksiyon CAN hattından yeni bir mesaj düştüğünde otomatik çalışır
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8]; // Gelen 8 bytelık veriyi tutacağımız dizi

    // FIFO'daki mesajı oku ve RxData dizisine kopyala
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
        // 1. Önce ID'yi kontrol edelim (Filtre zaten halletti ama çift dikiş olsun)
        if (RxHeader.Identifier == 0x11) 
        {
            // LED2'yi toggle yapalım, veri geldiğini gözle görelim
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1); 

            // 2. Byte 0'daki çalışma modunu oku
            uint8_t control_mode = RxData[0];

            // 3. Byte 1, 2, 3, 4'teki float veriyi birleştir (Union kullanarak)
            FloatConverter converter;
            converter.bytes[0] = RxData[1];
            converter.bytes[1] = RxData[2];
            converter.bytes[2] = RxData[3];
            converter.bytes[3] = RxData[4];

            // 4. Veriyi FOC Sürücü kütüphanemize besle
            if (control_mode == 1) // Tork Modu
            {
                motor1.config.current_ctrl_mode = true;
                motor1.input.T_mot_ref = converter.float_val; // Jetson'dan gelen tork değeri atandı!
            }
            else if (control_mode == 0) // Acil Durdurma / Boşa Çıkarma Modu
            {
                motor1.config.current_ctrl_mode = false;
                motor1.input.T_mot_ref = 0.0f;
                HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1); // LED2 Toggle (Acil durdurma geldiğinde de gözüksün diye)
            }
        }
    }
}

// =======================================================================================================================

// CAN HATTI TX
// Burada bir sıkıntı ortaya çıkıyor: 
// Biz CAN hattı üzerinden 4 farklı veri göndermek istiyoruz. HIZ - AKIM - BATARYA VOLTAJI - DURUM
// Fakat bu verilerin her biri 4 byte ve toplamda 16 byte yapar.
// Sorun ise CAN hattı sadece 8 byte veri taşıyabilir.
// Burada 8 byte ilk veri olarak sürekli değişen hız ve akım verilerini göndereceğiz.
// Diğer 8 byte'ta ise batarya voltajı ve durum bilgisini göndereceğiz.
// Batarya voltajı ve durum bilgisi sürekkli değişen değerler olmadığı için hız ve akım değerine göre saniye içindeki gönderimi daha az olacak.
// Yani hatta iki farklı paket basacağız. 
// Bu fonksiyon CAN hattına mesaj gönderildiğinde otomatik çalışır
void FDCAN_Send_Telemetry(void)
{
  // İki farklı paket için handle yapısı
    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    FloatConverter converter; 

    // 1. PAKET: HIZ VE AKIM (ID: 0x21)
    TxHeader.Identifier = 0x21; // Motor 1'in 1. Telemetri ID'si
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8; // 8 Byte yolluyoruz
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    // Hızı (Rad/s'den RPM'e çevirip) Union içine atıyoruz
    converter.float_val = motor1.input.w_rad_s * 9.54929658f;
    TxData[0] = converter.bytes[0];
    TxData[1] = converter.bytes[1];
    TxData[2] = converter.bytes[2];
    TxData[3] = converter.bytes[3];

    // Aktif q ekseni akımını (Tork üreten akım) Union içine atıyoruz
    converter.float_val = motor1.state.i_q;
    TxData[4] = converter.bytes[0];
    TxData[5] = converter.bytes[1];
    TxData[6] = converter.bytes[2];
    TxData[7] = converter.bytes[3];

    // CAN FIFO'ya ekleyip gönderiyoruz.
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);

    // 2. MESAJ: BATARYA VE DURUM (ID: 0x31)
    TxHeader.Identifier = 0x31; // Motor 1'in 2. Telemetri ID'si
    TxHeader.DataLength = FDCAN_DLC_BYTES_5; // Bu sefer 5 Byte yeterli

    // Batarya Voltajını Union içine atıyoruz
    converter.float_val = motor1.input.U_bat;
    TxData[0] = converter.bytes[0];
    TxData[1] = converter.bytes[1];
    TxData[2] = converter.bytes[2];
    TxData[3] = converter.bytes[3];

    // 5. Byte olarak da Hata Durumu veya Mevcut Hall Sektörünü gönderelim
    TxData[4] = HALL_GetCurrentSector(); 

    // CAN FIFO'ya ekleyip gönderiyoruz.
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
}

// =======================================================================================================================

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        // Gelen karakter Enter (\n veya \r) ise kelime bitmiştir
        if (rx_byte == '\n' || rx_byte == '\r') 
        {
            rx_buffer[rx_index] = '\0'; // String sonlandırıcı ekle
            command_ready = true;       // Ana döngüye "Komut hazır" bayrağını çek
            rx_index = 0;               // Bir sonraki kelime için başa sar
        } 
        else 
        {
            rx_buffer[rx_index++] = rx_byte; // Karakteri diziye ekle
            if (rx_index >= RX_BUFFER_SIZE) rx_index = 0; // Taşma koruması
        }
        
        // Bir sonraki karakteri dinlemek için kesmeyi tekrar kur
        HAL_UART_Receive_IT(&huart3, &rx_byte, 1); 
    }
}

// =======================================================================================================================

void Process_Debug_Command(void)
{
    if (!command_ready) return; // Yeni komut yoksa işlemciyi yorma, çık.

    // Komut P ile başlıyorsa (Proportional Gain)
    if (rx_buffer[0] == 'P') 
    {
        float new_kp = atof(&rx_buffer[1]); // P'den sonrasını float'a çevir
        motor1.config.Kp_q = new_kp;
        motor1.config.Kp_d = new_kp;
    }
    // Komut I ile başlıyorsa (Integral Gain)
    else if (rx_buffer[0] == 'I') 
    {
        float new_ki = atof(&rx_buffer[1]);
        motor1.config.Ki_q = new_ki;
        motor1.config.Ki_d = new_ki;
    }
    // Komut T ile başlıyorsa (Target Torque - Test İçin)
    else if (rx_buffer[0] == 'T') 
    {
        float new_tq = atof(&rx_buffer[1]);
        motor1.input.T_mot_ref = new_tq;
        motor1.config.current_ctrl_mode = true; // Motoru aktifleştir
    }
    // Komut S ise (Stop)
    else if (rx_buffer[0] == 'S') 
    {
        motor1.config.current_ctrl_mode = false;
        motor1.input.T_mot_ref = 0.0f;
    }

    command_ready = false; // Bayrağı indir, komut işlendi
}
/* USER CODE END 4 */


// =======================================================================================================================
// =======================================================================================================================
// =======================================================================================================================



/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
