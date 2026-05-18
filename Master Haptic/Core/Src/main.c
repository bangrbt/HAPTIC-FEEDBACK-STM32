#include "stm32f4xx.h"
#include <stdio.h>
#include <stdlib.h>

// ================= BIẾN TOÀN CỤC =================
volatile uint32_t ms_counter = 0;
volatile int16_t received_force = 0;
int32_t last_tx_pos = -999999;

// ================= PHẢN HỒI LỰC (FORCE FEEDBACK) =================
const float FORCE_DIRECTION = -1.0f; // Chiều phản lực tại Master (-1.0f hoặc 1.0f)
float force_ema = 0.0f;              // Lọc thông thấp phản lực
const float FORCE_ALPHA = 0.15f;      // Hệ số lọc (ALPHA càng nhỏ càng mượt)

// Khối biến giải mã Encoder Vi phân (Tuyệt đối không bị lỗi số âm)
volatile int32_t master_absolute_pos = 0;
uint16_t last_timer_cnt = 0;

char rx2_buffer[20];
volatile uint8_t rx2_idx = 0;

void Delay_ms(volatile uint32_t ms) {
    for (uint32_t i = 0; i < ms * 1600; i++) { __NOP(); }
}

void USART1_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    GPIOA->MODER &= ~(3UL << (9 * 2));
    GPIOA->MODER |= (2UL << (9 * 2));
    GPIOA->AFR[1] &= ~(0xFUL << 4);
    GPIOA->AFR[1] |= (7UL << 4);
    USART1->BRR = 0x008B;
    USART1->CR1 |= USART_CR1_UE | USART_CR1_TE;
}

void UART1_SendString(char* str) {
    while (*str) {
        while (!(USART1->SR & USART_SR_TXE)) {}
        USART1->DR = *str++;
    }
}

void USART2_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    GPIOA->MODER &= ~((3UL << (2 * 2)) | (3UL << (3 * 2)));
    GPIOA->MODER |= ((2UL << (2 * 2)) | (2UL << (3 * 2)));
    GPIOA->AFR[0] &= ~((0xFUL << (2 * 4)) | (0xFUL << (3 * 4)));
    GPIOA->AFR[0] |= ((7UL << (2 * 4)) | (7UL << (3 * 4)));
    USART2->BRR = 0x008B;
    USART2->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    NVIC_EnableIRQ(USART2_IRQn);
}

void UART2_SendString(char* str) {
    while (*str) {
        while (!(USART2->SR & USART_SR_TXE)) {}
        USART2->DR = *str++;
    }
}

void USART2_IRQHandler(void) {
    uint32_t sr = USART2->SR;
    // Xóa lỗi tràn để đường truyền không bị treo
    if (sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) {
        volatile uint32_t dummy = USART2->DR;
        (void)dummy;
        rx2_idx = 0;
    }
    if (sr & USART_SR_RXNE) {
        char c = USART2->DR;
        if (c == '\n') {
            rx2_buffer[rx2_idx] = '\0';
            if (rx2_buffer[0] == 'F') { received_force = atoi(&rx2_buffer[1]); }
            rx2_idx = 0;
        } else {
            if (rx2_idx < 19) rx2_buffer[rx2_idx++] = c;
        }
    }
}

void TIM2_Encoder_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    GPIOA->MODER &= ~((3UL << (0 * 2)) | (3UL << (1 * 2)));
    GPIOA->MODER |= ((2UL << (0 * 2)) | (2UL << (1 * 2)));
    GPIOA->AFR[0] &= ~((0xFUL << (0 * 4)) | (0xFUL << (1 * 4)));
    GPIOA->AFR[0] |= ((1UL << (0 * 4)) | (1UL << (1 * 4)));
    TIM2->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0 | (0xFUL << 4) | (0xFUL << 12);
    TIM2->SMCR  |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
    TIM2->ARR = 0xFFFFFFFF;
    TIM2->CR1 |= TIM_CR1_CEN;
}

void Motor_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    GPIOB->MODER &= ~((3UL << 0) | (3UL << 2));
    GPIOB->MODER |= ((1UL << 0) | (1UL << 2));
    GPIOA->MODER &= ~(3UL << 12);
    GPIOA->MODER |= (2UL << 12);
    GPIOA->AFR[0] &= ~(0xFUL << 24);
    GPIOA->AFR[0] |= (2UL << 24);
    TIM3->ARR = 1000 - 1;
    TIM3->CCMR1 |= (6UL << 4) | TIM_CCMR1_OC1PE;
    TIM3->CCER  |= TIM_CCER_CC1E;
    TIM3->CR1 |= TIM_CR1_CEN;
}

void Apply_Force_Feedback(int16_t force) {
    if (force > 700)  force = 700;
    if (force < -700) force = -700;
    if (force > 0) {
        GPIOB->BSRR = GPIO_BSRR_BR0 | GPIO_BSRR_BS1;
        TIM3->CCR1 = force;
    } else if (force < 0) {
        GPIOB->BSRR = GPIO_BSRR_BS0 | GPIO_BSRR_BR1;
        TIM3->CCR1 = -force;
    } else {
        GPIOB->BSRR = GPIO_BSRR_BR0 | GPIO_BSRR_BR1;
        TIM3->CCR1 = 0;
    }
}

void TIM4_Timer_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    TIM4->PSC = 16 - 1;
    TIM4->ARR = 1000 - 1; // Ngắt 1ms
    TIM4->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM4_IRQn);
    TIM4->CR1 |= TIM_CR1_CEN;
}

// Ngắt 1ms: Cập nhật vị trí âm/dương an toàn tuyệt đối
void TIM4_IRQHandler(void) {
    if (TIM4->SR & TIM_SR_UIF) {
        TIM4->SR &= ~TIM_SR_UIF;
        ms_counter++;

        // THUẬT TOÁN DELTA: Giải quyết triệt để lỗi không quay được chiều ngược lại
        uint16_t current_cnt = (uint16_t)TIM2->CNT;
        int16_t delta = (int16_t)(current_cnt - last_timer_cnt);
        master_absolute_pos += delta;
        last_timer_cnt = current_cnt;
    }
}

int main(void) {
    SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));
    USART1_Init();
    USART2_Init();
    TIM2_Encoder_Init();
    Motor_Init();

    // Đồng bộ biến đếm trước khi chạy
    last_timer_cnt = (uint16_t)TIM2->CNT;
    TIM4_Timer_Init();

    UART1_SendString("\r\n=== HAPTIC MASTER (2-WAY SYNC) ===\r\n");

    char tx_buf[64];
    uint32_t last_tx_time = 0;
    uint32_t last_debug_time = 0;

    while (1) {
        uint32_t current_time = ms_counter;
        int32_t current_pos = master_absolute_pos; // Dùng tọa độ chuẩn đã xử lý âm/dương

        // TẦNG 3: BỘ LỌC VÀ GIỚI HẠN AN TOÀN PHẢN LỰC MỀM
        force_ema = (FORCE_ALPHA * (float)received_force) + ((1.0f - FORCE_ALPHA) * force_ema);
        
        // Áp dụng hướng lực
        int16_t force_limited = (int16_t)(force_ema * FORCE_DIRECTION);
        
        // Giới hạn (Clamp) bảo vệ tuyệt đối cho tay người dùng và động cơ Master
        if (force_limited > 600)  force_limited = 600;
        if (force_limited < -600) force_limited = -600;
        
        Apply_Force_Feedback(force_limited);

        // Gửi tọa độ đi mỗi 10ms
        if (current_time - last_tx_time >= 10) {
            last_tx_time = current_time;
            if (current_pos != last_tx_pos) {
                sprintf(tx_buf, "P%ld\n", current_pos);
                UART2_SendString(tx_buf);
                last_tx_pos = current_pos;
            }
        }

        if (current_time - last_debug_time >= 300) {
            last_debug_time = current_time;
            sprintf(tx_buf, "Master Pos: %6ld | Force Rx: %4d\r\n", current_pos, received_force);
            UART1_SendString(tx_buf);
        }
    }
}
