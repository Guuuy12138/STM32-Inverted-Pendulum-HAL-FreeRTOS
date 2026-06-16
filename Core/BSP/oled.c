//
// Created by G on 2026/6/15.
// 移植自波特律动OLED驱动 (baud-dance OLED driver)
// 原始平台: TI MSPM0 → 目标平台: STM32F103 (HAL)
//

#include "oled.h"
#include "i2c.h"
#include <stdlib.h>

#define OLED_ADDR 0x3C

// OLED参数
#define OLED_PAGE 8            // OLED页数
#define OLED_ROW 8 * OLED_PAGE // OLED行数
#define OLED_COLUMN 128        // OLED列数

// 显存
uint8_t OLED_GRAM[OLED_PAGE][OLED_COLUMN];

// ========================== 底层通信函数 ==========================

/**
 * @brief 向OLED寄存器地址写一个byte的数据
 * @param addr 控制字节: 0x00=命令, 0x40=数据
 * @param data 要发送的数据/命令
 * @return 0=成功, -1=失败
 */
int OLED_WriteByte(uint8_t addr, uint8_t data)
{
  uint8_t buff[2];
  buff[0] = addr;
  buff[1] = data;

  if (HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR << 1, buff, 2, HAL_MAX_DELAY) == HAL_OK) {
    return 0;
  }
  return -1;
}

/**
 * @brief 向OLED发送数据流
 * @param data 数据缓冲区（首字节应为控制字节 0x40）
 * @param len  缓冲区长度
 */
void OLED_Send(uint8_t *data, uint8_t len) {
  HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR << 1, data, len, HAL_MAX_DELAY);
}

/**
 * @brief 向OLED发送指令
 */
void OLED_SendCmd(uint8_t cmd) {
  OLED_WriteByte(0x00, cmd);
}

// ========================== OLED驱动函数 ==========================

/**
 * @brief 初始化OLED (SSD1306)
 */
void OLED_Init(void) {
  HAL_Delay(200);      // 等待OLED上电稳定（原 delay_cycles(16000000) ≈ 200ms）
  OLED_SendCmd(0xAE);  // display off
  OLED_SendCmd(0x20);  // Set Memory Addressing Mode
  OLED_SendCmd(0x10);  // 00,Horizontal Addressing Mode;01,Vertical Addressing
                       // Mode;10,Page Addressing Mode (RESET);11,Invalid
  OLED_SendCmd(0xb0);  // Set Page Start Address for Page Addressing Mode,0-7
  OLED_SendCmd(0xc8);  // Set COM Output Scan Direction
  OLED_SendCmd(0x00);  //---set low column address
  OLED_SendCmd(0x10);  //---set high column address
  OLED_SendCmd(0x40);  //--set start line address
  OLED_SendCmd(0x81);  //--set contrast control register
  OLED_SendCmd(0xff);  // 对比度 0x00~0xff
  OLED_SendCmd(0xa1);  //--set segment re-map 0 to 127
  OLED_SendCmd(0xa6);  //--set normal display
  OLED_SendCmd(0xa8);  //--set multiplex ratio(1 to 64)
  OLED_SendCmd(0x3F);  //
  OLED_SendCmd(0xa4);  // 0xa4,Output follows RAM content;0xa5,Output ignores RAM content
  OLED_SendCmd(0xd3);  //-set display offset
  OLED_SendCmd(0x00);  //-not offset
  OLED_SendCmd(0xd5);  //--set display clock divide ratio/oscillator frequency
  OLED_SendCmd(0xf0);  //--set divide ratio
  OLED_SendCmd(0xd9);  //--set pre-charge period
  OLED_SendCmd(0x22);  //
  OLED_SendCmd(0xda);  //--set com pins hardware configuration
  OLED_SendCmd(0x12);
  OLED_SendCmd(0xdb);  //--set vcomh
  OLED_SendCmd(0x20);  // 0x20,0.77xVcc
  OLED_SendCmd(0x8d);  //--set DC-DC enable
  OLED_SendCmd(0x14);  //
  OLED_NewFrame();
  OLED_ShowFrame();
  OLED_SendCmd(0xaf);  //--turn on oled panel
}

/**
 * @brief 开启OLED显示
 */
void OLED_DisPlay_On(void) {
  OLED_SendCmd(0xAF);
  OLED_SendCmd(0x8D);
  OLED_SendCmd(0x14);
}

/**
 * @brief 关闭OLED显示
 */
void OLED_DisPlay_Off(void) {
  OLED_SendCmd(0xAE);
  OLED_SendCmd(0x8D);
  OLED_SendCmd(0x10);
}

/**
 * @brief 设置颜色模式 黑底白字或白底黑字
 * @param mode 颜色模式 COLOR_NORMAL / COLOR_REVERSED
 */
void OLED_SetColorMode(OLED_ColorMode mode) {
  if (mode == OLED_COLOR_NORMAL) {
    OLED_SendCmd(0xA6); // 正常显示
  }
  if (mode == OLED_COLOR_REVERSED) {
    OLED_SendCmd(0xA7); // 反色显示
  }
}

// ========================== 显存操作函数 ==========================

/**
 * @brief 清空显存 绘制新的一帧
 */
void OLED_NewFrame(void) {
  memset(OLED_GRAM, 0, sizeof(OLED_GRAM));
}

/**
 * @brief 将当前显存显示到屏幕上
 */
void OLED_ShowFrame(void) {
  static uint8_t sendBuffer[OLED_COLUMN + 1];
  sendBuffer[0] = 0x40;
  for (uint8_t i = 0; i < OLED_PAGE; i++) {
    OLED_SendCmd(0xB0 + i); // 设置页地址
    OLED_SendCmd(0x00);     // 设置列地址低4位
    OLED_SendCmd(0x10);     // 设置列地址高4位
    memcpy(sendBuffer + 1, OLED_GRAM[i], OLED_COLUMN);
    OLED_Send(sendBuffer, OLED_COLUMN + 1);
  }
}

/**
 * @brief 设置一个像素点
 * @param x 横坐标
 * @param y 纵坐标
 * @param color 颜色
 */
void OLED_SetPixel(uint8_t x, uint8_t y, OLED_ColorMode color) {
  if (x >= OLED_COLUMN || y >= OLED_ROW) return;
  if (!color) {
    OLED_GRAM[y / 8][x] |= 1 << (y % 8);
  } else {
    OLED_GRAM[y / 8][x] &= ~(1 << (y % 8));
  }
}

/**
 * @brief 设置显存中一字节数据的某几位
 * @param page 页地址
 * @param column 列地址
 * @param data 数据
 * @param start 起始位
 * @param end 结束位
 * @param color 颜色
 * @note start和end的范围为0-7, start必须小于等于end
 */
void OLED_SetByte_Fine(uint8_t page, uint8_t column, uint8_t data, uint8_t start, uint8_t end, OLED_ColorMode color) {
  static uint8_t temp;
  if (page >= OLED_PAGE || column >= OLED_COLUMN) return;
  if (color) data = ~data;

  temp = data | (0xff << (end + 1)) | (0xff >> (8 - start));
  OLED_GRAM[page][column] &= temp;
  temp = data & ~(0xff << (end + 1)) & ~(0xff >> (8 - start));
  OLED_GRAM[page][column] |= temp;
}

/**
 * @brief 设置显存中的一字节数据
 */
void OLED_SetByte(uint8_t page, uint8_t column, uint8_t data, OLED_ColorMode color) {
  if (page >= OLED_PAGE || column >= OLED_COLUMN) return;
  if (color) data = ~data;
  OLED_GRAM[page][column] = data;
}

/**
 * @brief 设置显存中的一字节数据的某几位（以像素坐标）
 * @note 可能出现跨两个真实字节的情况(跨页)
 */
void OLED_SetBits_Fine(uint8_t x, uint8_t y, uint8_t data, uint8_t len, OLED_ColorMode color) {
  uint8_t page = y / 8;
  uint8_t bit = y % 8;
  if (bit + len > 8) {
    OLED_SetByte_Fine(page, x, data << bit, bit, 7, color);
    OLED_SetByte_Fine(page + 1, x, data >> (8 - bit), 0, len + bit - 1 - 8, color);
  } else {
    OLED_SetByte_Fine(page, x, data << bit, bit, bit + len - 1, color);
  }
}

/**
 * @brief 设置显存中一字节长度的数据（以像素坐标）
 * @note 可能出现跨两个真实字节的情况(跨页)
 */
void OLED_SetBits(uint8_t x, uint8_t y, uint8_t data, OLED_ColorMode color) {
  uint8_t page = y / 8;
  uint8_t bit = y % 8;
  OLED_SetByte_Fine(page, x, data << bit, bit, 7, color);
  if (bit) {
    OLED_SetByte_Fine(page + 1, x, data >> (8 - bit), 0, bit - 1, color);
  }
}

/**
 * @brief 设置一块显存区域
 * @param data 列行式排列的像素数据
 */
void OLED_SetBlock(uint8_t x, uint8_t y, const uint8_t *data, uint8_t w, uint8_t h, OLED_ColorMode color) {
  uint8_t fullRow = h / 8;
  uint8_t partBit = h % 8;
  for (uint8_t i = 0; i < w; i++) {
    for (uint8_t j = 0; j < fullRow; j++) {
      OLED_SetBits(x + i, y + j * 8, data[i + j * w], color);
    }
  }
  if (partBit) {
    uint16_t fullNum = w * fullRow;
    for (uint8_t i = 0; i < w; i++) {
      OLED_SetBits_Fine(x + i, y + (fullRow * 8), data[fullNum + i], partBit, color);
    }
  }
}

// ========================== 图形绘制函数 ==========================

/**
 * @brief 绘制一条线段 (Bresenham算法)
 */
void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, OLED_ColorMode color) {
  static uint8_t temp = 0;
  if (x1 == x2) {
    if (y1 > y2) {
      temp = y1;
      y1 = y2;
      y2 = temp;
    }
    for (uint8_t y = y1; y <= y2; y++) {
      OLED_SetPixel(x1, y, color);
    }
  } else if (y1 == y2) {
    if (x1 > x2) {
      temp = x1;
      x1 = x2;
      x2 = temp;
    }
    for (uint8_t x = x1; x <= x2; x++) {
      OLED_SetPixel(x, y1, color);
    }
  } else {
    int16_t dx = x2 - x1;
    int16_t dy = y2 - y1;
    int16_t ux = ((dx > 0) << 1) - 1;
    int16_t uy = ((dy > 0) << 1) - 1;
    int16_t x = x1, y = y1, eps = 0;
    dx = abs(dx);
    dy = abs(dy);
    if (dx > dy) {
      for (x = x1; x != x2; x += ux) {
        OLED_SetPixel(x, y, color);
        eps += dy;
        if ((eps << 1) >= dx) {
          y += uy;
          eps -= dx;
        }
      }
    } else {
      for (y = y1; y != y2; y += uy) {
        OLED_SetPixel(x, y, color);
        eps += dx;
        if ((eps << 1) >= dy) {
          x += ux;
          eps -= dy;
        }
      }
    }
  }
}

/**
 * @brief 绘制一个矩形
 */
void OLED_DrawRectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h, OLED_ColorMode color) {
  OLED_DrawLine(x, y, x + w, y, color);
  OLED_DrawLine(x, y + h, x + w, y + h, color);
  OLED_DrawLine(x, y, x, y + h, color);
  OLED_DrawLine(x + w, y, x + w, y + h, color);
}

/**
 * @brief 绘制一个填充矩形
 */
void OLED_DrawFilledRectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h, OLED_ColorMode color) {
  for (uint8_t i = 0; i < h; i++) {
    OLED_DrawLine(x, y + i, x + w, y + i, color);
  }
}

/**
 * @brief 绘制一个三角形
 */
void OLED_DrawTriangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t x3, uint8_t y3, OLED_ColorMode color) {
  OLED_DrawLine(x1, y1, x2, y2, color);
  OLED_DrawLine(x2, y2, x3, y3, color);
  OLED_DrawLine(x3, y3, x1, y1, color);
}

/**
 * @brief 绘制一个填充三角形 (扫描线算法)
 */
void OLED_DrawFilledTriangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t x3, uint8_t y3, OLED_ColorMode color) {
  uint8_t a = 0, b = 0, y = 0, last = 0;
  if (y1 > y2) {
    a = y2;
    b = y1;
  } else {
    a = y1;
    b = y2;
  }
  y = a;
  for (; y <= b; y++) {
    if (y <= y3) {
      OLED_DrawLine(x1 + (y - y1) * (x2 - x1) / (y2 - y1), y, x1 + (y - y1) * (x3 - x1) / (y3 - y1), y, color);
    } else {
      last = y - 1;
      break;
    }
  }
  for (; y <= b; y++) {
    OLED_DrawLine(x2 + (y - y2) * (x3 - x2) / (y3 - y2), y, x1 + (y - last) * (x3 - x1) / (y3 - last), y, color);
  }
}

/**
 * @brief 绘制一个圆 (Bresenham中点画圆)
 */
void OLED_DrawCircle(uint8_t x, uint8_t y, uint8_t r, OLED_ColorMode color) {
  int16_t a = 0, b = r, di = 3 - (r << 1);
  while (a <= b) {
    OLED_SetPixel(x - b, y - a, color);
    OLED_SetPixel(x + b, y - a, color);
    OLED_SetPixel(x - a, y + b, color);
    OLED_SetPixel(x - b, y - a, color);
    OLED_SetPixel(x - a, y - b, color);
    OLED_SetPixel(x + b, y + a, color);
    OLED_SetPixel(x + a, y - b, color);
    OLED_SetPixel(x + a, y + b, color);
    OLED_SetPixel(x - b, y + a, color);
    a++;
    if (di < 0) {
      di += 4 * a + 6;
    } else {
      di += 10 + 4 * (a - b);
      b--;
    }
    OLED_SetPixel(x + a, y + b, color);
  }
}

/**
 * @brief 绘制一个填充圆 (Bresenham中点画圆)
 */
void OLED_DrawFilledCircle(uint8_t x, uint8_t y, uint8_t r, OLED_ColorMode color) {
  int16_t a = 0, b = r, di = 3 - (r << 1);
  while (a <= b) {
    for (int16_t i = x - b; i <= x + b; i++) {
      OLED_SetPixel(i, y + a, color);
      OLED_SetPixel(i, y - a, color);
    }
    for (int16_t i = x - a; i <= x + a; i++) {
      OLED_SetPixel(i, y + b, color);
      OLED_SetPixel(i, y - b, color);
    }
    a++;
    if (di < 0) {
      di += 4 * a + 6;
    } else {
      di += 10 + 4 * (a - b);
      b--;
    }
  }
}

/**
 * @brief 绘制一个椭圆
 */
void OLED_DrawEllipse(uint8_t x, uint8_t y, uint8_t a, uint8_t b, OLED_ColorMode color) {
  int xpos = 0, ypos = b;
  int a2 = a * a, b2 = b * b;
  int d = b2 + a2 * (0.25 - b);
  while (a2 * ypos > b2 * xpos) {
    OLED_SetPixel(x + xpos, y + ypos, color);
    OLED_SetPixel(x - xpos, y + ypos, color);
    OLED_SetPixel(x + xpos, y - ypos, color);
    OLED_SetPixel(x - xpos, y - ypos, color);
    if (d < 0) {
      d = d + b2 * ((xpos << 1) + 3);
      xpos += 1;
    } else {
      d = d + b2 * ((xpos << 1) + 3) + a2 * (-(ypos << 1) + 2);
      xpos += 1, ypos -= 1;
    }
  }
  d = b2 * (xpos + 0.5) * (xpos + 0.5) + a2 * (ypos - 1) * (ypos - 1) - a2 * b2;
  while (ypos > 0) {
    OLED_SetPixel(x + xpos, y + ypos, color);
    OLED_SetPixel(x - xpos, y + ypos, color);
    OLED_SetPixel(x + xpos, y - ypos, color);
    OLED_SetPixel(x - xpos, y - ypos, color);
    if (d < 0) {
      d = d + b2 * ((xpos << 1) + 2) + a2 * (-(ypos << 1) + 3);
      xpos += 1, ypos -= 1;
    } else {
      d = d + a2 * (-(ypos << 1) + 3);
      ypos -= 1;
    }
  }
}

/**
 * @brief 绘制一张图片
 */
void OLED_DrawImage(uint8_t x, uint8_t y, const Image *img, OLED_ColorMode color) {
  OLED_SetBlock(x, y, img->data, img->w, img->h, color);
}

// ================================ 文字绘制 ================================

/**
 * @brief 绘制一个ASCII字符
 */
void OLED_PrintASCIIChar(uint8_t x, uint8_t y, char ch, const ASCIIFont *font, OLED_ColorMode color) {
  OLED_SetBlock(x, y, font->chars + (ch - ' ') * (((font->h + 7) / 8) * font->w), font->w, font->h, color);
}

/**
 * @brief 绘制一个ASCII字符串
 */
void OLED_PrintASCIIString(uint8_t x, uint8_t y, char *str, const ASCIIFont *font, OLED_ColorMode color) {
  uint8_t x0 = x;
  while (*str) {
    OLED_PrintASCIIChar(x0, y, *str, font, color);
    x0 += font->w;
    str++;
  }
}

/**
 * @brief 获取UTF-8编码的字符长度
 */
static uint8_t _OLED_GetUTF8Len(char *string) {
  if ((string[0] & 0x80) == 0x00) {
    return 1;
  } else if ((string[0] & 0xE0) == 0xC0) {
    return 2;
  } else if ((string[0] & 0xF0) == 0xE0) {
    return 3;
  } else if ((string[0] & 0xF8) == 0xF0) {
    return 4;
  }
  return 0;
}

/**
 * @brief 绘制字符串（支持UTF-8中文 + ASCII回落）
 * @note 编译器字符集需设置为UTF-8
 * @note 字库使用波特律动LED取模工具生成(https://led.baud-dance.com)
 */
void OLED_PrintString(uint8_t x, uint8_t y, char *str, const Font *font, OLED_ColorMode color) {
  uint16_t i = 0;
  uint8_t oneLen = (((font->h + 7) / 8) * font->w) + 4; // 一个字模占多少字节
  uint8_t found;
  uint8_t utf8Len;
  uint8_t *head;

  while (str[i]) {
    found = 0;
    utf8Len = _OLED_GetUTF8Len(str + i);
    if (utf8Len == 0) break;

    // 在线性字库中查找字符
    for (uint8_t j = 0; j < font->len; j++) {
      head = (uint8_t *)(font->chars) + (j * oneLen);
      if (memcmp(str + i, head, utf8Len) == 0) {
        OLED_SetBlock(x, y, head + 4, font->w, font->h, color);
        x += font->w;
        i += utf8Len;
        found = 1;
        break;
      }
    }

    // 未找到字模: ASCII字符回落显示, 非ASCII显示空格
    if (found == 0) {
      if (utf8Len == 1) {
        OLED_PrintASCIIChar(x, y, str[i], font->ascii, color);
        x += font->ascii->w;
        i += utf8Len;
      } else {
        OLED_PrintASCIIChar(x, y, ' ', font->ascii, color);
        x += font->ascii->w;
        i += utf8Len;
      }
    }
  }
}
