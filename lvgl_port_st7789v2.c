#include "main.h"
#include "lvgl.h"

#define MY_DISP_HOR_RES 240
#define MY_DISP_VER_RES 320

#ifndef LCD_ROTATE
#define LCD_ROTATE 0
#endif

#if !(LCD_ROTATE==0) && !(LCD_ROTATE==1) && !(LCD_ROTATE==2) && !(LCD_ROTATE==3)
#define LCD_ROTATE 0
#endif



SPI_HandleTypeDef LCD_SPI;

#define LCD_IO(_NAME, _VAL) HAL_GPIO_WritePin(LCD_##_NAME##_GPIO_Port,LCD_##_NAME##_Pin,_VAL)
#define LCD_CMD(_val) do{uint8_t val = _val;LCD_IO(DC,0);LCD_IO(CS,0); HAL_SPI_Transmit(&LCD_SPI,&val,1,HAL_MAX_DELAY);LCD_IO(CS,1);}while(0);
#define LCD_DATA(_val) do{uint8_t val = _val;LCD_IO(DC,1);LCD_IO(CS,0); HAL_SPI_Transmit(&LCD_SPI,&val,1,HAL_MAX_DELAY);LCD_IO(CS,1);}while(0);

#define MV (1<<5)
#define MX (1<<6)
#define MY (1<<7)
static void st7789v2_init() {
    LCD_IO(CS,1);
    LCD_IO(RST,0);
    HAL_Delay(100);
    LCD_IO(RST,1);
    HAL_Delay(100);

    LCD_CMD(0x11); 			//Sleep Out
    HAL_Delay(120);         //DELAY120ms
    //--------------------------------ST7789S Frame rate setting----------------------------------//
    LCD_CMD(0x2a); 		//Column address set
    LCD_DATA(0x00); 		//start column
    LCD_DATA(0x00);
    LCD_DATA(0x00);		//end column
    LCD_DATA(0xef);

    LCD_CMD(0x2b); 		//Row address set
    LCD_DATA(0x00); 		//start row
    LCD_DATA(0x28);
    LCD_DATA(0x01);		//end row
    LCD_DATA(0x17);

    LCD_CMD(0xb2); 		//Porch control
    LCD_DATA(0x0c);
    LCD_DATA(0x0c);
    LCD_DATA(0x00);
    LCD_DATA(0x33);
    LCD_DATA(0x33);

    LCD_CMD(0x20); 		//Display Inversion Off

    LCD_CMD(0xb7); 		//Gate control
    LCD_DATA(0x56);   		//35
//---------------------------------ST7789S Power setting--------------------------------------//
    LCD_CMD(0xbb); //VCOMS Setting
    LCD_DATA(0x18);  //1f

    LCD_CMD(0xc0); 		//LCM Control
    LCD_DATA(0x2c);

    LCD_CMD(0xc2); 		//VDV and VRH Command Enable
    LCD_DATA(0x01);

    LCD_CMD(0xc3); //VRH Set
    LCD_DATA(0x1f); //12

    LCD_CMD(0xc4); 			//VDV Setting
    LCD_DATA(0x20);

    LCD_CMD(0xc6); 			//FR Control 2
    LCD_DATA(0x0f);
//LCD_CMD(0xca);
//LCD_DATA(0x0f);
//LCD_CMD(0xc8);
//LCD_DATA(0x08);
//LCD_CMD(0x55);
//LCD_DATA(0x90);
    LCD_CMD(0xd0);  //Power Control 1
    LCD_DATA(0xa6);   //a4
    LCD_DATA(0xa1);
//--------------------------------ST7789S gamma setting---------------------------------------//

    LCD_CMD(0xe0);
    LCD_DATA(0xd0);
    LCD_DATA(0x0d);
    LCD_DATA(0x14);
    LCD_DATA(0x0b);
    LCD_DATA(0x0b);
    LCD_DATA(0x07);
    LCD_DATA(0x3a);
    LCD_DATA(0x44);
    LCD_DATA(0x50);
    LCD_DATA(0x08);
    LCD_DATA(0x13);
    LCD_DATA(0x13);
    LCD_DATA(0x2d);
    LCD_DATA(0x32);

    LCD_CMD(0xe1); 				//Negative Voltage Gamma Contro
    LCD_DATA(0xd0);
    LCD_DATA(0x0d);
    LCD_DATA(0x14);
    LCD_DATA(0x0b);
    LCD_DATA(0x0b);
    LCD_DATA(0x07);
    LCD_DATA(0x3a);
    LCD_DATA(0x44);
    LCD_DATA(0x50);
    LCD_DATA(0x08);
    LCD_DATA(0x13);
    LCD_DATA(0x13);
    LCD_DATA(0x2d);
    LCD_DATA(0x32);

    LCD_CMD(0x36); 			//Memory data access control
#if (LCD_ROTATE == 0)
    LCD_DATA(0)
#elif (LCD_ROTATE == 1)
    LCD_DATA(MV|MX);
#elif LCD_ROTATE == 2
    LCD_DATA(MY|MX);
#elif LCD_ROTATE == 3
    LCD_DATA(MV|MY)
#endif

    LCD_CMD(0x3A); 			//Interface pixel format
    LCD_DATA(0x55);			//65K
//    LCD_DATA(0x66);			//262K  RGB 6 6 6

    LCD_CMD(0xe7); 			//SPI2 enable    启用2数据通道模式
    LCD_DATA(0x00);


    LCD_CMD(0x21);			//Display inversion on
    LCD_CMD(0x29); 			//Display on
}
static void st7789v2_set_window(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2) {
    LCD_CMD(0x2a);        //Column address set
    LCD_DATA(x1>>8);        //start row
    LCD_DATA(x1);
    LCD_DATA(x2>>8);        //end row
    LCD_DATA(x2);


    LCD_CMD(0x2b);        //Row address set
    LCD_DATA(y1>>8);        //start column
    LCD_DATA(y1);
    LCD_DATA(y2>>8);        //end column
    LCD_DATA(y2);

}
static void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {

    st7789v2_set_window(area->x1,area->y1,area->x2,area->y2);

    LCD_CMD(0x2C);            //Memory write

    LCD_SPI.Init.DataSize = SPI_DATASIZE_16BIT;
    HAL_SPI_Init(&LCD_SPI);
    uint16_t size = (area->y2-area->y1+1)*(area->x2-area->x1+1);
    LCD_IO(DC,1);
    LCD_IO(CS,0);
    HAL_SPI_Transmit(&LCD_SPI,(uint8_t*)color_p,size,HAL_MAX_DELAY);
    LCD_IO(CS,1);

    LCD_SPI.Init.DataSize = SPI_DATASIZE_8BIT;
    HAL_SPI_Init(&LCD_SPI);
   lv_disp_flush_ready(disp);         /* Indicate you are ready with the flushing*/
}

void st7789v2_clear(uint16_t color) {

#if (LCD_ROTATE==0) || (LCD_ROTATE==2)
    st7789v2_set_window(0,0,MY_DISP_HOR_RES-1,MY_DISP_VER_RES-1);
#else
    st7789v2_set_window(0,0,MY_DISP_VER_RES-1,MY_DISP_HOR_RES-1);
#endif
    LCD_CMD(0x2C);            //Memory write

    LCD_SPI.Init.DataSize = SPI_DATASIZE_16BIT;
    HAL_SPI_Init(&LCD_SPI);
    LCD_IO(DC,1);
    LCD_IO(CS,0);
    for (int i = 0; i < 240; ++i) {
        for (int j = 0; j < 320; ++j) {
            HAL_SPI_Transmit(&LCD_SPI,(uint8_t*)&color,1,HAL_MAX_DELAY);
        }
    }
    LCD_IO(CS,1);

    LCD_SPI.Init.DataSize = SPI_DATASIZE_8BIT;
    HAL_SPI_Init(&LCD_SPI);
}


void lvgl_port_st7789v2() {

    st7789v2_init();

    static lv_disp_draw_buf_t draw_buf;

#if (LCD_ROTATE==0) || (LCD_ROTATE==2)
    static lv_color_t buf1[MY_DISP_HOR_RES * 20];                        /*Declare a buffer for 1/10 screen size*/
//    static lv_color_t buf2[MY_DISP_HOR_RES * 20];                        /*Declare a buffer for 1/10 screen size*/
    lv_disp_draw_buf_init(&draw_buf, buf1, NULL,
                          MY_DISP_HOR_RES * 20);  /*Initialize the display buffer.*/
#else
    static lv_color_t buf1[MY_DISP_VER_RES * 10];                        /*Declare a buffer for 1/10 screen size*/
//    static lv_color_t buf2[MY_DISP_VER_RES * 20];                        /*Declare a buffer for 1/10 screen size*/
    lv_disp_draw_buf_init(&draw_buf, buf1, NULL,
                          MY_DISP_VER_RES * 10);  /*Initialize the display buffer.*/
#endif


    static lv_disp_drv_t disp_drv;               /*Descriptor of a display driver*/
    lv_disp_drv_init(&disp_drv);          /*Basic initialization*/
    disp_drv.flush_cb = my_disp_flush;    /*Set your driver function*/
    disp_drv.draw_buf = &draw_buf;          /*Assign the buffer to the display*/
    disp_drv.hor_res = MY_DISP_HOR_RES;   /*Set the horizontal resolution of the display*/
    disp_drv.ver_res = MY_DISP_VER_RES;   /*Set the verizontal resolution of the display*/
    disp_drv.rotated = LCD_ROTATE;
    lv_disp_drv_register(&disp_drv);      /*Finally register the driver*/

}
