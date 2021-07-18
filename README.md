# 说明
基于 STM32 HAL 库，为 `lvgl` 提供 `st7789v2`控制器支持。
- 颜色格式：RGB656

# 硬件设置
## IO
- ST7789V2_DC：输出
- ST7789V2_CS：输出
- ST7789V2_RST：输出
## SPI
- ST7789V2_SPI：默认（或改分频）

## 其他
- ST7789V2_ROTATE：显示方向
    - LV_DISP_ROT_NONE(default)
    - LV_DISP_ROT_90
    - LV_DISP_ROT_180
    - LV_DISP_ROT_270




# CMake 引入

```CMake

add_subdirectory(stm32cube) # required by lvgl_port_st7789v2
add_subdirectory(3rdparty/lvgl)      # required by lvgl_port_st7789v2
add_subdirectory(drivers/lvgl_port_st7789v2)

add_executable(app.elf main.c)
target_link_libraries(app.elf
    PRIVATE
    stm32cube::core
    lvgl
    lvgl::port::st7789v2)

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

