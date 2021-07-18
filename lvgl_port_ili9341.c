#include "lvgl.h"
#include "main.h"

#define MY_DISP_HOR_RES 240
#define MY_DISP_VER_RES 320

#ifndef LCD_ROTATE
#define LCD_ROTATE 0
#endif

#if !(LCD_ROTATE == 0) && !(LCD_ROTATE == 1) && !(LCD_ROTATE == 2) &&          \
    !(LCD_ROTATE == 3)
#define LCD_ROTATE 0
#endif

SPI_HandleTypeDef LCD_SPI;

#define LCD_IO(_NAME, _VAL)                                                    \
  HAL_GPIO_WritePin(LCD_##_NAME##_GPIO_Port, LCD_##_NAME##_Pin, _VAL)
#define LCD_CMD(_val)                                                          \
  do {                                                                         \
    uint8_t val = _val;                                                        \
    LCD_IO(DC, 0);                                                             \
    LCD_IO(CS, 0);                                                             \
    HAL_SPI_Transmit(&LCD_SPI, &val, 1, HAL_MAX_DELAY);                        \
    LCD_IO(CS, 1);                                                             \
  } while (0);
#define LCD_DATA(_val)                                                         \
  do {                                                                         \
    uint8_t val = _val;                                                        \
    LCD_IO(DC, 1);                                                             \
    LCD_IO(CS, 0);                                                             \
    HAL_SPI_Transmit(&LCD_SPI, &val, 1, HAL_MAX_DELAY);                        \
    LCD_IO(CS, 1);                                                             \
  } while (0);

#define MV (1 << 5)
#define MX (1 << 6)
#define MY (1 << 7)
static void ili9341_init() {
  LCD_IO(CS, 1);
  LCD_IO(RST, 0);
  HAL_Delay(100);
  LCD_IO(RST, 1);
  HAL_Delay(100);

  LCD_CMD(0xCB); // 功耗控制A
  LCD_DATA(0x39);
  LCD_DATA(0x2C);
  LCD_DATA(0x00);
  LCD_DATA(0x34); // 内核电压控制
  LCD_DATA(0x02); // DDVDH控制

  LCD_CMD(0xCF); // 功耗控制B
  LCD_DATA(0x00);
  LCD_DATA(0XC1); // Power 控制
  LCD_DATA(0X30);

  LCD_CMD(0xE8);  // 驱动时序控制A
  LCD_DATA(0x85); // 栅极驱动器的非重叠时序控制
  LCD_DATA(0x00); // EQ 时序控制
  LCD_DATA(0x78); // 预充电时间控制

  LCD_CMD(0xEA);  // 驱动时序控制B
  LCD_DATA(0x00); // 栅极驱动器时序控制
  LCD_DATA(0x00);

  LCD_CMD(0xED);  // 电源序列控制
  LCD_DATA(0x64); // 软启动控制
  LCD_DATA(0x03); // 电源序列控制
  LCD_DATA(0X12); // 电源序列控制
  LCD_DATA(0X81); // DDVDH增强模式

  LCD_CMD(0xF7);  // 泵比控制
  LCD_DATA(0x20); // 比率控制

  LCD_CMD(0xC0);  // Power control // 功耗控制1
  LCD_DATA(0x1b); // VRH[5:0] // 设置 GVDD电平

  LCD_CMD(0xC1);  // Power control // 功耗控制2
  LCD_DATA(0x10); // SAP[2:0];BT[3:0] // 设置用于升压电路的因子

  LCD_CMD(0xC5);  // VCM control //VCOM控制1
  LCD_DATA(0x2d); // 设置 VCOMH电压
  LCD_DATA(0x33); // 设置 VCOML电压

  // LCD_CMD(0xC7); //VCM control2 //VCOM控制2
  // LCD_DATA(0xCf); // 设置 VCOM偏移电压

  LCD_CMD(0x36); // Memory Access Control // 存储器访问控制
#if (LCD_ROTATE == 0)
  LCD_DATA(0x08 | MX)
#elif LCD_ROTATE == 1
  LCD_DATA(MV | 0x08);
#elif LCD_ROTATE == 2
  LCD_DATA(MY | 0x08);
#elif LCD_ROTATE == 3
  LCD_DATA(MV | MY | MX | 0x08)
#endif

  LCD_CMD(0xB1);  //( 正常模式 / 全色模式下 ) 帧速率控制
  LCD_DATA(0x00); // 内部时钟分频设置， 00时表示不分频
  LCD_DATA(0x1d); // RTNA 设置，用于设置 1H(行) 的时间

  LCD_CMD(0xB6); // Display Function Control // 显示功能设置
  LCD_DATA(0x0A); // 设置在没显示区域的扫描格式， 0A表示间隔扫描
  LCD_DATA(0x02); // 设置源极、栅极驱动器的移动方向和扫描周期

  LCD_CMD(0xF2);  // 3Gamma Function Disable // 使能 3G
  LCD_DATA(0x00); // 01 使能 3G，00不使能

  LCD_CMD(0x26);  // Gamma curve selected // 伽马设置
  LCD_DATA(0x01); // 选择伽马曲线1

  LCD_CMD(0xE0); // Set Gamma // 正极伽马校准
  LCD_DATA(0x0F);
  LCD_DATA(0x3a);
  LCD_DATA(0x36);
  LCD_DATA(0x0b);
  LCD_DATA(0x0d);
  LCD_DATA(0x06);
  LCD_DATA(0x4c);
  LCD_DATA(0x91);
  LCD_DATA(0x31);
  LCD_DATA(0x08);
  LCD_DATA(0x10);
  LCD_DATA(0x04);
  LCD_DATA(0x11);
  LCD_DATA(0x0c);
  LCD_DATA(0x00);

  LCD_CMD(0XE1); // Set Gamma // 负极伽马校准
  LCD_DATA(0x00);
  LCD_DATA(0x06);
  LCD_DATA(0x0a);
  LCD_DATA(0x05);
  LCD_DATA(0x12);
  LCD_DATA(0x09);
  LCD_DATA(0x2c);
  LCD_DATA(0x92);
  LCD_DATA(0x3f);
  LCD_DATA(0x08);
  LCD_DATA(0x0e);
  LCD_DATA(0x0b);
  LCD_DATA(0x2e);
  LCD_DATA(0x33);
  LCD_DATA(0x0F);

  LCD_CMD(0x3A); //数据模式设置默认0x66（18bit） 0x55 16bit
  LCD_DATA(0x55);

  LCD_CMD(0x21); //正常显示

  LCD_CMD(0x11); // Exit Sleep // 退出睡眠模式
  HAL_Delay(120);
  LCD_CMD(0x29); // Display on // 开显示
}
static void ili9341_set_window(uint16_t x1, uint16_t y1, uint16_t x2,
                               uint16_t y2) {
  LCD_CMD(0x2a);     // Column address set
  LCD_DATA(x1 >> 8); // start row
  LCD_DATA(x1);
  LCD_DATA(x2 >> 8); // end row
  LCD_DATA(x2);

  LCD_CMD(0x2b);     // Row address set
  LCD_DATA(y1 >> 8); // start column
  LCD_DATA(y1);
  LCD_DATA(y2 >> 8); // end column
  LCD_DATA(y2);
}
static void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area,
                          lv_color_t *color_p) {

  ili9341_set_window(area->x1, area->y1, area->x2, area->y2);

  LCD_CMD(0x2C); // Memory write

  LCD_SPI.Init.DataSize = SPI_DATASIZE_16BIT;
  HAL_SPI_Init(&LCD_SPI);
  uint16_t size = (area->y2 - area->y1 + 1) * (area->x2 - area->x1 + 1);
  LCD_IO(DC, 1);
  LCD_IO(CS, 0);
  HAL_SPI_Transmit(&LCD_SPI, (uint8_t *)color_p, size, HAL_MAX_DELAY);
  LCD_IO(CS, 1);

  LCD_SPI.Init.DataSize = SPI_DATASIZE_8BIT;
  HAL_SPI_Init(&LCD_SPI);
  lv_disp_flush_ready(disp); /* Indicate you are ready with the flushing*/
}

void ili9341_clear(uint16_t color) {

#if (LCD_ROTATE == 0) || (LCD_ROTATE == 2)
  ili9341_set_window(0, 0, MY_DISP_HOR_RES - 1, MY_DISP_VER_RES - 1);
#else
  ili9341_set_window(0, 0, MY_DISP_VER_RES - 1, MY_DISP_HOR_RES - 1);
#endif
  LCD_CMD(0x2C); // Memory write

  LCD_SPI.Init.DataSize = SPI_DATASIZE_16BIT;
  HAL_SPI_Init(&LCD_SPI);
  LCD_IO(DC, 1);
  LCD_IO(CS, 0);
  for (int i = 0; i < 240; ++i) {
    for (int j = 0; j < 320; ++j) {
      HAL_SPI_Transmit(&LCD_SPI, (uint8_t *)&color, 1, HAL_MAX_DELAY);
    }
  }
  LCD_IO(CS, 1);

  LCD_SPI.Init.DataSize = SPI_DATASIZE_8BIT;
  HAL_SPI_Init(&LCD_SPI);
}

void lvgl_port_ili9341() {

  ili9341_init();

  static lv_disp_draw_buf_t draw_buf;

#if (LCD_ROTATE == 0) || (LCD_ROTATE == 2)
  static lv_color_t
      buf1[MY_DISP_HOR_RES * 20]; /*Declare a buffer for 1/10 screen size*/
  //    static lv_color_t buf2[MY_DISP_HOR_RES * 20]; /*Declare a buffer for
  //    1/10 screen size*/
  lv_disp_draw_buf_init(&draw_buf, buf1, NULL,
                        MY_DISP_HOR_RES *
                            20); /*Initialize the display buffer.*/
#else
  static lv_color_t
      buf1[MY_DISP_VER_RES * 10]; /*Declare a buffer for 1/10 screen size*/
  //    static lv_color_t buf2[MY_DISP_VER_RES * 20]; /*Declare a buffer for
  //    1/10 screen size*/
  lv_disp_draw_buf_init(&draw_buf, buf1, NULL,
                        MY_DISP_VER_RES *
                            10); /*Initialize the display buffer.*/
#endif

  static lv_disp_drv_t disp_drv;     /*Descriptor of a display driver*/
  lv_disp_drv_init(&disp_drv);       /*Basic initialization*/
  disp_drv.flush_cb = my_disp_flush; /*Set your driver function*/
  disp_drv.draw_buf = &draw_buf;     /*Assign the buffer to the display*/
  disp_drv.hor_res =
      MY_DISP_HOR_RES; /*Set the horizontal resolution of the display*/
  disp_drv.ver_res =
      MY_DISP_VER_RES; /*Set the verizontal resolution of the display*/
  disp_drv.rotated = LCD_ROTATE;
  lv_disp_drv_register(&disp_drv); /*Finally register the driver*/
}
