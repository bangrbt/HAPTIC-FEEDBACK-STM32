#ifndef SSD1306_I2C_H
#define SSD1306_I2C_H

#include "stm32f4xx.h"
#include <stdint.h>

// Định nghĩa địa chỉ I2C của màn hình
#define OLED_ADDR 0x78

// Khai báo các hàm khởi tạo và điều khiển I2C
void I2C1_Init(void);
void I2C_WriteReg(uint8_t dev_addr, uint8_t control_byte, uint8_t data);

// Khai báo các hàm điều khiển OLED
void OLED_Cmd(uint8_t cmd);
void OLED_Data(uint8_t data);
void OLED_SetCursor(uint8_t page, uint8_t col);
void OLED_Clear(void);
void OLED_PrintChar(char c);
void OLED_PrintString(uint8_t page, uint8_t col, char* str);
void OLED_Init(void);

#endif /* SSD1306_I2C_H */
