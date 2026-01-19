/**
 * @file lv_port_disp.c
 *
 */

/* 1. 开启文件：从 0 改为 1 */
#if 1

/*********************
 * INCLUDES
 *********************/
#include "lv_port_disp.h"
#include <stdbool.h>

/* TODO: 【Wait for you】请引入你的 LCD 驱动头文件，比如 "bsp_lcd.h" */
#include "lcd_init.h" 
#include "lcd.h"


/*********************
 * DEFINES
 *********************/
/* 2. 修改分辨率：适配 240x320 */
#ifndef MY_DISP_HOR_RES
    #define MY_DISP_HOR_RES    240
#endif

#ifndef MY_DISP_VER_RES
    #define MY_DISP_VER_RES    320
#endif

/* 定义缓冲区高度 (行数)
 * 40行是一个很好的平衡点。
 * 显存占用: 240 * 40 * 2byte = 19.2 KB (HC32F460 RAM够大，放心用)
 */
#define DISP_BUF_HEIGHT    100

/**********************
 * TYPEDEFS
 **********************/

/**********************
 * STATIC PROTOTYPES
 **********************/
static void disp_init(void);

static void disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p);

/**********************
 * STATIC VARIABLES
 **********************/

/**********************
 * MACROS
 **********************/

/**********************
 * GLOBAL FUNCTIONS
 **********************/

void lv_port_disp_init(void)
{
    /*-------------------------
     * Initialize your display
     * -----------------------*/
    disp_init();

    /*-----------------------------
     * Create a buffer for drawing
     *----------------------------*/
    
    /* 3. 配置缓冲区 (使用最稳健的“单缓冲区 + 局部刷新”模式) */
    static lv_disp_draw_buf_t draw_buf_dsc_1;
    
    /* 申请缓冲区内存 (放在内部 RAM 即可) */
    static lv_color_t buf_1[MY_DISP_HOR_RES * DISP_BUF_HEIGHT];  

    /* 初始化绘制缓冲区 */
    lv_disp_draw_buf_init(&draw_buf_dsc_1, buf_1, NULL, MY_DISP_HOR_RES * DISP_BUF_HEIGHT);   

    /*-----------------------------------
     * Register the display in LVGL
     *----------------------------------*/

    static lv_disp_drv_t disp_drv;                  /*Descriptor of a display driver*/
    lv_disp_drv_init(&disp_drv);                    /*Basic initialization*/

    /*Set the resolution of the display*/
    disp_drv.hor_res = MY_DISP_HOR_RES;
    disp_drv.ver_res = MY_DISP_VER_RES;

    /*Used to copy the buffer's content to the display*/
    disp_drv.flush_cb = disp_flush;

    /*Set a display buffer*/
    disp_drv.draw_buf = &draw_buf_dsc_1;

    /*Finally register the driver*/
    lv_disp_drv_register(&disp_drv);
}

/**********************
 * STATIC FUNCTIONS
 **********************/

/*Initialize your display and the required peripherals.*/
static void disp_init(void)
{
    /* TODO: 【Wait for you】在这里调用你的 LCD 初始化函数 */
    LCD_Init(); 
}

volatile bool disp_flush_enabled = true;

/* Enable updating the screen (the flushing process) when disp_flush() is called by LVGL
 */
void disp_enable_update(void)
{
    disp_flush_enabled = true;
}

/* Disable updating the screen (the flushing process) when disp_flush() is called by LVGL
 */
void disp_disable_update(void)
{
    disp_flush_enabled = false;
}

/* 4. 核心刷屏函数：对接你的 DMA */
static void disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
    if(disp_flush_enabled) {
        /* 计算当前刷新的区域宽高 */
        uint16_t width = area->x2 - area->x1 + 1;
        uint16_t height = area->y2 - area->y1 + 1;

        /* TODO: 【Wait for you】调用你的 DMA 刷图函数 
         * 参数假设是: x, y, width, height, 数据指针
         * 注意: color_p 是 lv_color_t 类型，需要强转为 uint8_t* 传给你的底层
         */
        LCD_ShowImage_DMA(area->x1, area->y1, width, height, (uint8_t *)color_p);
    }

    /* IMPORTANT!!! 
     * 必须通知 LVGL 这一帧数据已经处理完了
     * 如果你的 DMA 函数是“阻塞式”的(等传完才返回)，直接在这里调 Ready 即可。
     */
    lv_disp_flush_ready(disp_drv);
}

#else /*Enable this file at the top*/

/*This dummy typedef exists purely to silence -Wpedantic.*/
typedef int keep_pedantic_happy;
#endif