# 说明

基于 STM32 HAL 库，为 `lvgl` 提供 LCD 控制器支持。

- 颜色格式：RGB656
- 支持控制器
  - st7789v2
  - ili9341

# 硬件设置

## IO

- LCD_DC：输出
- LCD_CS：输出
- LCD_RST：输出

## SPI

- LCD_SPI：输出主模式（或改分频）

## 其他

- LCD_ROTATE：显示方向
  - 0(default)：正常
  - 1：90 度
  - 2：180 度
  - 3：270 度

# CMake 引入

```CMake

add_subdirectory(stm32cube) # required by lvgl_port_lcd
add_subdirectory(3rdparty/lvgl)      # required by lvgl_port_lcd
add_subdirectory(drivers/lvgl_port_lcd)

add_executable(app.elf main.c)
target_link_libraries(app.elf
    PRIVATE
    stm32cube::core
    lvgl
    lvgl::port::st7789v2
    lvgl::port::ili9341)


```

# 使用

```C++
#include "lvgl.h"
#include "lvgl_port_st7789v2.h"

void setup() {
    lvgl_init();
    lvgl_port_st7789v2();
}
```
