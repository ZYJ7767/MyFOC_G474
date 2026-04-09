#ifndef __LCD_H
#define __LCD_H

#include "main.h"
#include "cmsis_os2.h"

/* ---------------- 硬件接口 (与 CubeMX 保持一致) ---------------- */
#define LCD_PWR_PORT    GPIOD
#define LCD_PWR_PIN     GPIO_PIN_8
#define LCD_RST_PORT    GPIOD
#define LCD_RST_PIN     GPIO_PIN_9
#define LCD_CS_PORT     GPIOD
#define LCD_CS_PIN      GPIO_PIN_10
#define LCD_DC_PORT     GPIOB
#define LCD_DC_PIN      GPIO_PIN_4

#define LCD_PWR(x)   HAL_GPIO_WritePin(LCD_PWR_PORT, LCD_PWR_PIN, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define LCD_RST(x)   HAL_GPIO_WritePin(LCD_RST_PORT, LCD_RST_PIN, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define LCD_CS(x)    HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define LCD_DC(x)    HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)

/* ---------------- 常用的 RGB565 颜色 ---------------- */
#define LCD_WIDTH    240
#define LCD_HEIGHT   240

#define LCD_WHITE    0xFFFF
#define LCD_BLACK    0x0000
#define LCD_BLUE     0x001F  
#define LCD_RED      0xF800
#define LCD_GREEN    0x07E0
#define LCD_YELLOW   0xFFE0

/* ---------------- 开放给 FreeRTOS 的 API ---------------- */
void LCD_Init(void);
void LCD_Clear(uint16_t color);

// 通过 DMA 发送显存数组给屏幕
void LCD_Fill_DMA(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t *color_buffer);

// 本地纯内存字模渲染 (不会触发SPI，专门为DMA打配合)
// buf_w 是二维数组在X轴的实际宽度，buf_h 固定为 16
void LCD_Draw_String_To_Buffer(char *str, uint16_t *buf, uint16_t buf_w, uint16_t buf_h, uint16_t color, uint16_t bg_color);
void LCD_Draw_String_To_Buffer_X2(char *str, uint16_t *buf, uint16_t buf_w, uint16_t buf_h, uint16_t color, uint16_t bg_color);
#endif /* __LCD_H */
