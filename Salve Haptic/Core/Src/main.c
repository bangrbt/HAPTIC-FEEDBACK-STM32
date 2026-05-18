#include "stm32f4xx.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "ssd1306_i2c.h"

volatile uint32_t ms_counter = 0;
volatile int32_t target_pos = 0;
int16_t force_tx = 0;
int16_t last_force_tx = 0;
float v_zero = 2.5f;
float current_ema = 0.0f;
const float ALPHA = 0.02f;

// ================= BỘ THÔNG SỐ PID =================
float Kp = 25.0f;
float Ki = 0.08f;
float Kd = 45.0f;
float error_sum = 0.0f;
float last_error = 0.0f;

// ================= BỘ LỌC TÍN HIỆU =================
float pos_ema = 0.0f;
uint8_t pos_ema_initialized = 0;
const float POS_ALPHA = 0.8f;

float last_filtered_derivative = 0.0f;
const float DERIVATIVE_ALPHA = 0.6f;

// ================= HỆ THỐNG PHẢN HỒI LỰC =================
volatile float F_feedback = 0.0f;
const float I_friction = 0.15f;
const float I_deadband = 0.15f;
const float K_force = 1500.0f;

const int8_t MOTOR_DIRECTION = -1;

char rx2_buffer[20];
volatile uint8_t rx2_idx = 0;

void Delay_ms(volatile uint32_t ms) {
    for (uint32_t i = 0; i < ms * 1600; i++) { __NOP(); }
}

// ================= CẤU HÌNH CÒI BÁO (BUZZER) TẠI PA8 =================
void Buzzer_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    // Cấu hình PA8 làm chân phát PWM (TIM1_CH1)
    GPIOA->MODER &= ~(3UL << (8 * 2));
    GPIOA->MODER |= (2UL << (8 * 2));
    
    // CỰC KỲ QUAN TRỌNG: Cấu hình Open-Drain để khắc phục lỗi còi 5V kêu liên tục
    GPIOA->OTYPER |= (1UL << 8); 

    GPIOA->AFR[1] &= ~(0xFUL << ((8 - 8) * 4));
    GPIOA->AFR[1] |= (1UL << ((8 - 8) * 4));

    TIM1->PSC = 16 - 1;       // Tần số base 1 MHz
    TIM1->ARR = 1000 - 1;     // Tần số còi 1 kHz (Tiếng bíp thanh)
    TIM1->CCMR1 |= (6UL << 4) | TIM_CCMR1_OC1PE; // Chế độ PWM
    TIM1->CCER |= TIM_CCER_CC1E;
    TIM1->BDTR |= TIM_BDTR_MOE;  // Kích hoạt đầu ra chính cho TIM1
    TIM1->CR1 |= TIM_CR1_CEN;
    TIM1->CCR1 = 1000;        // Tắt còi ban đầu (Buzzer Active Low cần PWM 100% High để tắt)
}

void USART1_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    GPIOA->MODER &= ~(3UL << (9 * 2));   GPIOA->MODER |= (2UL << (9 * 2));
    GPIOA->AFR[1] &= ~(0xFUL << 4);      GPIOA->AFR[1] |= (7UL << 4);
    USART1->BRR = 0x008B; USART1->CR1 |= USART_CR1_UE | USART_CR1_TE;
}
void UART1_SendString(char* str) {
    while (*str) {
        while (!(USART1->SR & USART_SR_TXE)) {}
        USART1->DR = *str++;
    }
}

void UART2_SendString(char* str) {
    while (*str) {
        while (!(USART2->SR & USART_SR_TXE)) {}
        USART2->DR = *str++;
    }
}

void USART2_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    GPIOA->MODER &= ~((3UL << (2 * 2)) | (3UL << (3 * 2))); GPIOA->MODER |= ((2UL << (2 * 2)) | (2UL << (3 * 2)));
    GPIOA->AFR[0] &= ~((0xFUL << (2 * 4)) | (0xFUL << (3 * 4))); GPIOA->AFR[0] |= ((7UL << (2 * 4)) | (7UL << (3 * 4)));
    USART2->BRR = 0x008B; USART2->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    NVIC_EnableIRQ(USART2_IRQn);
}

void USART2_IRQHandler(void) {
    uint32_t sr = USART2->SR;
    if (sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) {
        volatile uint32_t dummy = USART2->DR; (void)dummy; rx2_idx = 0;
    }
    if (sr & USART_SR_RXNE) {
        char c = USART2->DR;
        if (c == '\n') {
            rx2_buffer[rx2_idx] = '\0';
            if (rx2_buffer[0] == 'P') { target_pos = atol(&rx2_buffer[1]); }
            rx2_idx = 0;
        } else {
            if (rx2_idx < 19) rx2_buffer[rx2_idx++] = c;
        }
    }
}

void TIM2_Encoder_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    GPIOA->MODER &= ~((3UL << (0 * 2)) | (3UL << (1 * 2))); GPIOA->MODER |= ((2UL << (0 * 2)) | (2UL << (1 * 2)));
    GPIOA->AFR[0] &= ~((0xFUL << (0 * 4)) | (0xFUL << (1 * 4))); GPIOA->AFR[0] |= ((1UL << (0 * 4)) | (1UL << (1 * 4)));
    TIM2->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0 | (0xFUL << 4) | (0xFUL << 12);
    TIM2->SMCR  |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1; TIM2->ARR = 0xFFFFFFFF; TIM2->CR1 |= TIM_CR1_CEN;
}

void Motor_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN; RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    GPIOB->MODER &= ~((3UL << 0) | (3UL << 2)); GPIOB->MODER |= ((1UL << 0) | (1UL << 2));
    GPIOA->MODER &= ~(3UL << 12);               GPIOA->MODER |= (2UL << 12);
    GPIOA->AFR[0] &= ~(0xFUL << 24);            GPIOA->AFR[0] |= (2UL << 24);
    TIM3->ARR = 1000 - 1; TIM3->CCMR1 |= (6UL << 4) | TIM_CCMR1_OC1PE; TIM3->CCER  |= TIM_CCER_CC1E; TIM3->CR1 |= TIM_CR1_CEN;
}

void Motor_SetSpeed(int16_t duty) {
    duty = duty * MOTOR_DIRECTION;

    if (duty > 1000)  duty = 1000;
    if (duty < -1000) duty = -1000;

    if (duty > 0) {
        GPIOB->BSRR = GPIO_BSRR_BS0 | GPIO_BSRR_BR1; TIM3->CCR1 = duty;
    } else if (duty < 0) {
        GPIOB->BSRR = GPIO_BSRR_BR0 | GPIO_BSRR_BS1; TIM3->CCR1 = -duty;
    } else {
        GPIOB->BSRR = GPIO_BSRR_BR0 | GPIO_BSRR_BR1; TIM3->CCR1 = 0;
    }
}

void ADC1_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    GPIOA->MODER |= (3UL << (4 * 2)); ADC1->SQR3 = 4; ADC1->CR2 |= ADC_CR2_ADON; Delay_ms(10);
}

uint16_t ADC1_Read(void) {
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC)) {} return ADC1->DR;
}

void Calibrate_ACS712(void) {
    UART1_SendString("Dang hieu chuan cam bien dong...\r\n");
    float sum_v_acs = 0;

    Motor_SetSpeed(0);
    Delay_ms(100);

    for(int i = 0; i < 200; i++) {
        uint16_t adc_raw = ADC1_Read();
        float v_pin = ((float)adc_raw / 4095.0f) * 3.3f;
        float v_acs = v_pin * 1.5f;
        sum_v_acs += v_acs;
        Delay_ms(1);
    }

    v_zero = sum_v_acs / 200.0f;

    char buf[50];
    sprintf(buf, "Hieu chuan xong! V_Zero = %.3f V\r\n", v_zero);
    UART1_SendString(buf);
}

void TIM4_Timer_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    TIM4->PSC = 16 - 1; TIM4->ARR = 2000 - 1;
    TIM4->DIER |= TIM_DIER_UIE; NVIC_EnableIRQ(TIM4_IRQn); TIM4->CR1 |= TIM_CR1_CEN;
}

void TIM4_IRQHandler(void) {
    if (TIM4->SR & TIM_SR_UIF) {
        TIM4->SR &= ~TIM_SR_UIF;
        ms_counter += 2;

        int32_t raw_pos = (int32_t)TIM2->CNT;
        
        if (!pos_ema_initialized) {
            pos_ema = (float)raw_pos;
            pos_ema_initialized = 1;
        }
        pos_ema = (POS_ALPHA * (float)raw_pos) + ((1.0f - POS_ALPHA) * pos_ema);
        
        int32_t current_pos = (int32_t)pos_ema;
        float error = (float)(target_pos - current_pos);

        if ((error > 0.0f && last_error < 0.0f) || (error < 0.0f && last_error > 0.0f)) {
            error_sum = 0.0f; 
        }

        if (error < 30.0f && error > -30.0f) {
            error_sum += error;
        } else {
            error_sum = 0.0f;
        }
        
        if (error_sum > 1000.0f)  error_sum = 1000.0f;
        if (error_sum < -1000.0f) error_sum = -1000.0f;

        float raw_derivative = error - last_error;
        float filtered_derivative = (DERIVATIVE_ALPHA * raw_derivative) + ((1.0f - DERIVATIVE_ALPHA) * last_filtered_derivative);
        
        float output_pwm = (Kp * error) + (Ki * error_sum) + (Kd * filtered_derivative);
        last_error = error;
        last_filtered_derivative = filtered_derivative;

        if (error > 5.0f && output_pwm < 80.0f)  output_pwm += 80.0f;
        if (error < -5.0f && output_pwm > -80.0f) output_pwm -= 80.0f;

        if (error > 2500.0f || error < -2500.0f) {
            Motor_SetSpeed(0);
        } else {
            Motor_SetSpeed((int16_t)output_pwm);
        }

        uint16_t adc_raw = ADC1_Read();
        float v_pin = ((float)adc_raw / 4095.0f) * 3.3f;
        float v_acs = v_pin * 1.5f;

        float current_raw = (v_acs - v_zero) / 0.185f;

        if (current_raw > -0.12f && current_raw < 0.12f) {
            current_raw = 0.0f;
        }

        current_ema = (ALPHA * current_raw) + ((1.0f - ALPHA) * current_ema);

        float I_motor = fabs(current_ema);
        float I_contact = I_motor - I_friction;
        if (I_contact < I_deadband) {
            I_contact = 0.0f;
        }

        float sign_pwm = (output_pwm > 0.0f) ? 1.0f : ((output_pwm < 0.0f) ? -1.0f : 0.0f);
        float F_calc = K_force * I_contact * sign_pwm;

        if (F_calc > 700.0f)  F_calc = 700.0f;
        if (F_calc < -700.0f) F_calc = -700.0f;
        F_feedback = F_calc;

        // ================= KÍCH HOẠT CÒI BÁO ĐỘNG TẠI ĐÂY =================
        float force_abs = fabs(F_feedback);
        if (force_abs > 600.0f) {
            // Chỉ kêu khi lực cản cực kỳ lớn (> 400)
            // Lực từ 400 -> 700 (khoảng 300 đơn vị) sẽ ép CCR1 từ 1000 -> 0 (Kêu to dần)
            int ccr_val = 1000 - (int)((force_abs - 600.0f) * 3.33f);
            if (ccr_val < 0) ccr_val = 0;
            TIM1->CCR1 = (uint16_t)ccr_val;
        } else {
            TIM1->CCR1 = 1000; // Tắt hoàn toàn còi nếu không vượt ngưỡng 400
        }
    }
}

int main(void) {
    SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));
    char uart_buf[64]; char oled_buf[20];
    uint32_t last_display_time = 0;
    uint32_t last_force_tx_time = 0;

    USART1_Init(); USART2_Init(); TIM2_Encoder_Init(); Motor_Init();
    ADC1_Init(); I2C1_Init(); OLED_Init();

    Buzzer_Init(); // Kích hoạt phần cứng Còi báo

    OLED_Clear(); OLED_PrintString(0, 0, "BRobionic System");
    
    Calibrate_ACS712();

    OLED_Clear();
    OLED_PrintString(0, 0, "I_mot:");
    OLED_PrintString(2, 0, "Set :"); OLED_PrintString(4, 0, "Real:"); OLED_PrintString(6, 0, "Err :");

    TIM4_Timer_Init();

    while (1) {
        uint32_t current_time = ms_counter;

        if (current_time - last_force_tx_time >= 20) {
            last_force_tx_time = current_time;
            char force_buf[32];
            sprintf(force_buf, "F%d\n", (int16_t)F_feedback);
            UART2_SendString(force_buf);
        }

        if (current_time - last_display_time >= 300) {
            last_display_time = current_time;
            int32_t current_pos = (int32_t)TIM2->CNT;
            int32_t err_display = target_pos - current_pos;

            int cur_ma_tx = (int)(current_ema * 1000.0f);
            sprintf(uart_buf, "Tgt: %6ld | Cur: %6ld | Err: %5ld | ImA: %5d\r\n", target_pos, current_pos, err_display, cur_ma_tx);
            UART1_SendString(uart_buf);

            int cur_ma = (int)(current_ema * 1000.0f);
            sprintf(oled_buf, "%4d mA   ", cur_ma); OLED_PrintString(0, 42, oled_buf);

            sprintf(oled_buf, "%ld      ", target_pos); OLED_PrintString(2, 36, oled_buf);
            sprintf(oled_buf, "%ld      ", current_pos); OLED_PrintString(4, 36, oled_buf);
            sprintf(oled_buf, "%ld      ", err_display); OLED_PrintString(6, 36, oled_buf);
        }
    }
}
