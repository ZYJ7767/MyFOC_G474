#include "LCD_SPI.h"
#include "spi.h"
#include "LCD_Font.h"

extern SPI_HandleTypeDef hspi1;
static osSemaphoreId_t lcd_dma_sem = NULL;

static void LCD_Write_Cmd(uint8_t cmd) {
    LCD_DC(0); LCD_CS(0);
    HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);
    LCD_CS(1);
}

static void LCD_Write_Data(uint8_t data) {
    LCD_DC(1); LCD_CS(0);
    HAL_SPI_Transmit(&hspi1, &data, 1, 100);
    LCD_CS(1);
}

static void LCD_Set_Window(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    LCD_Write_Cmd(0x2A);
    LCD_Write_Data(x1 >> 8); LCD_Write_Data(x1 & 0xFF);
    LCD_Write_Data(x2 >> 8); LCD_Write_Data(x2 & 0xFF);
    LCD_Write_Cmd(0x2B);
    LCD_Write_Data(y1 >> 8); LCD_Write_Data(y1 & 0xFF);
    LCD_Write_Data(y2 >> 8); LCD_Write_Data(y2 & 0xFF);
}

void LCD_Init(void)
{
    // 创建二值信号量，用于DMA同步
    lcd_dma_sem = osSemaphoreNew(1, 0, NULL);
    
    // 开机先把背光拉低，防止初始化乱闪
    LCD_PWR(0); 
    LCD_CS(1); 
    
    // 硬件复位
    LCD_RST(0);
    osDelay(50); // RTOS主动出让CPU
    LCD_RST(1);
    osDelay(120);

    /* --- 寄存器底裤 --- */
    
    LCD_Write_Cmd(0x11);     // Sleep Out
    osDelay(120);            // 唤醒必须老老实实等 120ms
    
    LCD_Write_Cmd(0x36);     // Memory Data Access Control
    LCD_Write_Data(0x00);
    
    LCD_Write_Cmd(0x3A);     // 像素格式 (原厂用的65)
    LCD_Write_Data(0x65);

    LCD_Write_Cmd(0xB2);     // Porch Setting
    LCD_Write_Data(0x0C);
    LCD_Write_Data(0x0C);
    LCD_Write_Data(0x00);
    LCD_Write_Data(0x33);
    LCD_Write_Data(0x33);

    LCD_Write_Cmd(0xB7);     // Gate Control
    LCD_Write_Data(0x72);

    LCD_Write_Cmd(0xBB);     // VCOM Setting
    LCD_Write_Data(0x3D);

    LCD_Write_Cmd(0xC0);     // LCM Control
    LCD_Write_Data(0x2C);

    LCD_Write_Cmd(0xC2);     // VDV and VRH Command Enable
    LCD_Write_Data(0x01);

    LCD_Write_Cmd(0xC3);     // VRH Set
    LCD_Write_Data(0x19);

    LCD_Write_Cmd(0xC4);     // VDV Set
    LCD_Write_Data(0x20);

    LCD_Write_Cmd(0xC6);     // Frame Rate Control in Normal Mode
    LCD_Write_Data(0x0F);

    LCD_Write_Cmd(0xD0);     // Power Control 1
    LCD_Write_Data(0xA4);
    LCD_Write_Data(0xA1);

    LCD_Write_Cmd(0xE0);     // 正电压 Gamma 控制
    LCD_Write_Data(0xD0); LCD_Write_Data(0x04); LCD_Write_Data(0x0D);
    LCD_Write_Data(0x11); LCD_Write_Data(0x13); LCD_Write_Data(0x2B);
    LCD_Write_Data(0x3F); LCD_Write_Data(0x54); LCD_Write_Data(0x4C);
    LCD_Write_Data(0x18); LCD_Write_Data(0x0D); LCD_Write_Data(0x0B);
    LCD_Write_Data(0x1F); LCD_Write_Data(0x23);

    LCD_Write_Cmd(0xE1);     // 负电压 Gamma 控制
    LCD_Write_Data(0xD0); LCD_Write_Data(0x04); LCD_Write_Data(0x0C);
    LCD_Write_Data(0x11); LCD_Write_Data(0x13); LCD_Write_Data(0x2C);
    LCD_Write_Data(0x3F); LCD_Write_Data(0x44); LCD_Write_Data(0x51);
    LCD_Write_Data(0x2F); LCD_Write_Data(0x1F); LCD_Write_Data(0x1F);
    LCD_Write_Data(0x20); LCD_Write_Data(0x23);

    LCD_Write_Cmd(0x21);     // 屏幕反色 (1.3寸必备)
    
    LCD_Write_Cmd(0x29);     // Display On
    osDelay(10);
    
    LCD_PWR(1);
}

void LCD_Fill_DMA(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t *color_buffer) {
    
    uint32_t pixels = (xend - xsta + 1) * (yend - ysta + 1);
    LCD_Set_Window(xsta, ysta, xend, yend);
    LCD_Write_Cmd(0x2C);
    LCD_DC(1); LCD_CS(0); 
    
    // 启动基于 8-bit SPI 的16位颜色搬运 (总字节数 = 像素数 * 2)
    HAL_SPI_Transmit_DMA(&hspi1, (uint8_t *)color_buffer, pixels * 2);
    osSemaphoreAcquire(lcd_dma_sem, osWaitForever);
    LCD_CS(1);
}

// 纯内存在线渲染 (自动大小端转换、自动涂满背景色、速度达极微秒级)
void LCD_Draw_String_To_Buffer(char *str, uint16_t *buf, uint16_t buf_w, uint16_t buf_h, uint16_t color, uint16_t bg_color)
{
    uint16_t cursor_x = 0;
    // SPI 是 8 bit, 颜色必须高低字节对调！
    uint16_t swap_c = (color >> 8) | (color << 8);
    uint16_t swap_bg = (bg_color >> 8) | (bg_color << 8);

    // 1. 初始化显存的背景色，防止数据残留花块
    for (int i = 0; i < buf_w * buf_h; i++) {
        buf[i] = swap_bg;
    }

    // 2. 遍历字符串，把字模画进这个数组 (绝不调用硬件操作)
    while(*str != '\0') {
        if(cursor_x + 8 > buf_w) break;
        uint8_t c = *str - ' '; 
        if(c > 94) c = 0; // 防越界
        
        for(uint8_t y = 0; y < 16; y++) {
            if(y >= buf_h) break;
            uint8_t line = asc2_1608[c][y]; 
            for(uint8_t x = 0; x < 8; x++) {
                if((line & (0x80 >> x)) != 0) {
                    buf[y * buf_w + cursor_x + x] = swap_c;
                }
            }
        }
        cursor_x += 8;
        str++;
    }
}

void LCD_Draw_String_To_Buffer_X2(char *str, uint16_t *buf, uint16_t buf_w, uint16_t buf_h, uint16_t color, uint16_t bg_color)
{
    uint16_t cursor_x = 0;
    uint16_t swap_c = (color >> 8) | (color << 8);
    uint16_t swap_bg = (bg_color >> 8) | (bg_color << 8);

    // 初始化背景
    for (int i = 0; i < buf_w * buf_h; i++) buf[i] = swap_bg;

    // 映射 8x16 字体到 16x32 (1个点变4个像素点)
    while(*str != '\0') {
        if(cursor_x + 16 > buf_w) break; 
        uint8_t c = *str - ' ';
        if(c > 94) c = 0;

        for(uint8_t y = 0; y < 16; y++) {
            if(y * 2 >= buf_h) break;
            uint8_t line = asc2_1608[c][y];
            for(uint8_t x = 0; x < 8; x++) {
                if((line & (0x80 >> x)) != 0) {
                    buf[(y * 2) * buf_w + cursor_x + x * 2] = swap_c;
                    buf[(y * 2) * buf_w + cursor_x + x * 2 + 1] = swap_c;
                    buf[(y * 2 + 1) * buf_w + cursor_x + x * 2] = swap_c;
                    buf[(y * 2 + 1) * buf_w + cursor_x + x * 2 + 1] = swap_c;
                }
            }
        }
        cursor_x += 16;
        str++;
    }
}


void LCD_Clear(uint16_t color) {
    uint16_t line_buf[LCD_WIDTH];
    uint16_t swap_color = (color >> 8) | (color << 8); 
    for (int i = 0; i < LCD_WIDTH; i++) line_buf[i] = swap_color; 
    for (int y = 0; y < LCD_HEIGHT; y++) LCD_Fill_DMA(0, y, LCD_WIDTH - 1, y, line_buf);
}

// DMA 传输完成回调中断
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    if(hspi->Instance == SPI1) {
        if (lcd_dma_sem != NULL) {
            osSemaphoreRelease(lcd_dma_sem); 
        }
    }
}
