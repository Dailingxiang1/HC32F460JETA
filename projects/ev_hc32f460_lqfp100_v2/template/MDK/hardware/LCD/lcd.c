// lcd.c
#include "lcd.h"
#include "lcd_init.h"
#include "lcdfont.h"
#include "main.h"
void LCD_Fill(u16 xsta, u16 ysta, u16 xend, u16 yend, u16 color)
{
    uint32_t total_point = (uint32_t)(xend - xsta) * (uint32_t)(yend - ysta);
    uint8_t  hi = (uint8_t)(color >> 8);
    uint8_t  lo = (uint8_t)(color & 0xFF);
    LCD_Address_Set(xsta, ysta, (u16)(xend - 1), (u16)(yend - 1));

    LCD_CS_Clr();
    LCD_DC_Set();   /* data */

    for (uint32_t i = 0; i < total_point; i++)
    {
        /* 发高字节 */

        while (RESET == SPI_GetStatus(SPI_UNIT, SPI_FLAG_TX_BUF_EMPTY)) {
            ;
        }
        SPI_WriteData(SPI_UNIT, (uint16_t)hi);

        /* 发低字节 */
        while (RESET == SPI_GetStatus(SPI_UNIT, SPI_FLAG_TX_BUF_EMPTY)) {
            ;
        }
        SPI_WriteData(SPI_UNIT, (uint16_t)lo);
    }

    /* ? 等最后一笔真正发送完成：
       1) TX缓冲空
       2) SPI空闲(IDLE)
     */
    while (RESET == SPI_GetStatus(SPI_UNIT, SPI_FLAG_TX_BUF_EMPTY)) {
        ;
    }
    while (RESET == SPI_GetStatus(SPI_UNIT, SPI_FLAG_IDLE)) {
        ;
    }

    LCD_CS_Set();
}
/**
 * @brief  配置并启动一次 DMA 传输
 * @param  srcAddr: 源数据地址 (数组指针)
 * @param  count:   本次传输的字节数 (最大 65535)
 */
static void LCD_DMA_Start(const uint8_t *srcAddr, uint32_t count)
{
    /* 1. 必须先关闭 DMA 通道才能修改配置 */
    //DMA_Cmd(DMA_UNIT, ENABLE); // 确保DMA总开关是开的
    DMA_ChCmd(DMA_UNIT, DMA_TX_CH, DISABLE);
    
    /* 2. 清除之前的传输完成标志 (非常重要，否则刚开启就进中断或认为完成了) */
    //DMA_ClearTransCompleteStatus(DMA_UNIT, DMA_TX_CH);

    /* 3.由此处重新配置源地址和传输长度 */
    DMA_SetSrcAddr(DMA_UNIT, DMA_TX_CH, (uint32_t)srcAddr);
    DMA_SetTransCount(DMA_UNIT, DMA_TX_CH, count);
    
    /* 4. 开启通道，传输立即开始 (因为 SPI TX 触发源一直连着) */
    DMA_ChCmd(DMA_UNIT, DMA_TX_CH, ENABLE);
	
		SPI_Cmd(SPI_UNIT, ENABLE);
}
extern volatile uint8_t g_dma_transfer_complete;
/**
 * @brief  DMA 刷屏函数
 * @note   请确保 pData 里的字节序已经是 [高, 低, 高, 低...]，DMA 不会帮你交换！
 */
void LCD_ShowImage_DMA(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t *pData)
{
    uint32_t total_bytes = width * height * 2;
    uint32_t sent_bytes = 0;
    uint32_t current_block_size;

    LCD_Address_Set(x, y, x + width - 1, y + height - 1);
    LCD_CS_Clr();
    LCD_DC_Set();

    /* 初始化变量，防止第一次误判 */
    g_dma_transfer_complete = 1;

    while (sent_bytes < total_bytes)
    {
        /* 1. 计算块大小 */
        if ((total_bytes - sent_bytes) > 60000)
             current_block_size = 60000;
        else
             current_block_size = total_bytes - sent_bytes;

        /* 2. 等待上一块完成 (依赖中断修改变量) */
        while(g_dma_transfer_complete == 0);
        g_dma_transfer_complete = 0; // 清零，准备发下一块

        /* 3. 配置 DMA (必须先 Disable) */
        DMA_ChCmd(DMA_UNIT, DMA_TX_CH, DISABLE);
        
        /* 注意：中断里已经清除了标志位，这里不需要再清，但为了保险可以再清一次 */
        // DMA_ClearTransCompleteStatus(DMA_UNIT, DMA_TX_CH);

        DMA_SetSrcAddr(DMA_UNIT, DMA_TX_CH, (uint32_t)(pData + sent_bytes));
        DMA_SetTransCount(DMA_UNIT, DMA_TX_CH, current_block_size);
        
        /* 4. 开启 DMA */
        DMA_ChCmd(DMA_UNIT, DMA_TX_CH, ENABLE);
        
        /* 5. 【AOS软件触发】 踢一脚 */
        /* 前提：你的初始化代码里配置了 COMEN 和 INTSFTTRG */
        AOS_SW_Trigger();
        
        /* 6. 循环继续... CPU可以在这里干别的事，等待中断把 g_dma_transfer_complete 置 1 */
        
        sent_bytes += current_block_size;
    }
    
    /* 7. 等待最后一块完成 */
    while(g_dma_transfer_complete == 0);

    /* 等待 SPI 发送空闲 */
    while (RESET == SPI_GetStatus(SPI_UNIT, SPI_FLAG_TX_BUF_EMPTY));
    while (RESET == SPI_GetStatus(SPI_UNIT, SPI_FLAG_IDLE));
    LCD_CS_Set();
}

/**
 * @brief  显示图片 (RGB565格式)
 * @param  x      起始X坐标
 * @param  y      起始Y坐标
 * @param  width  图片宽度
 * @param  height 图片高度
 * @param  pData  图片数据数组指针 (由LVGL Converter生成的uint8_t数组)
 */
void LCD_ShowImage(u16 x, u16 y, u16 width, u16 height, const uint8_t *pData)
{
    /* 1. 计算总像素点数 */
    uint32_t total_point = (uint32_t)width * (uint32_t)height;
    
    /* 2. 设置显示区域窗口 
       注意：跟你的LCD_Fill一样，终点坐标通常是 起点+宽度-1 */
    LCD_Address_Set(x, y, x + width - 1, y + height - 1);

    /* 3. 准备SPI传输 */
    LCD_CS_Clr();
    LCD_DC_Set();   /* Data command */

    /* 4. 循环发送像素数据 */
    for (uint32_t i = 0; i < total_point; i++)
    {
        /* 获取当前像素的高字节和低字节
           假设数组格式为: [Hi0, Lo0, Hi1, Lo1, ...] 
           如果显示的颜色不对（反色或错乱），可能需要交换 pData[i*2] 和 pData[i*2+1] 的位置
        */
        uint8_t hi = pData[i * 2 + 1];     // 偶数索引为高字节
        uint8_t lo = pData[i * 2 ]; // 奇数索引为低字节

        /* --- 发送高字节 --- */
        while (RESET == SPI_GetStatus(SPI_UNIT, SPI_FLAG_TX_BUF_EMPTY)) {
            ; // 等待发送缓冲为空
        }
        SPI_WriteData(SPI_UNIT, (uint16_t)hi);

        /* --- 发送低字节 --- */
        while (RESET == SPI_GetStatus(SPI_UNIT, SPI_FLAG_TX_BUF_EMPTY)) {
            ; // 等待发送缓冲为空
        }
        SPI_WriteData(SPI_UNIT, (uint16_t)lo);
    }

    /* 5. 等待最后的传输完成并释放片选 */
    while (RESET == SPI_GetStatus(SPI_UNIT, SPI_FLAG_TX_BUF_EMPTY)) {
        ;
    }
    while (RESET == SPI_GetStatus(SPI_UNIT, SPI_FLAG_IDLE)) {
        ;
    }

    LCD_CS_Set();
}

/******************************************************************************
      函数说明：在指定位置画点
      入口数据：x,y 画点坐标
                color 点的颜色
      返回值：  无
******************************************************************************/
void LCD_DrawPoint(u16 x,u16 y,u16 color)
{
	LCD_Address_Set(x,y,x,y);//设置光标位置 
	LCD_WR_DATA(color);
} 


/******************************************************************************
      函数说明：画线
      入口数据：x1,y1   起始坐标
                x2,y2   终止坐标
                color   线的颜色
      返回值：  无
******************************************************************************/
void LCD_DrawLine(u16 x1,u16 y1,u16 x2,u16 y2,u16 color)
{
	u16 t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance;
	int incx,incy,uRow,uCol;
	delta_x=x2-x1; //计算坐标增量 
	delta_y=y2-y1;
	uRow=x1;//画线起点坐标
	uCol=y1;
	if(delta_x>0)incx=1; //设置单步方向 
	else if (delta_x==0)incx=0;//垂直线 
	else {incx=-1;delta_x=-delta_x;}
	if(delta_y>0)incy=1;
	else if (delta_y==0)incy=0;//水平线 
	else {incy=-1;delta_y=-delta_y;}
	if(delta_x>delta_y)distance=delta_x; //选取基本增量坐标轴 
	else distance=delta_y;
	for(t=0;t<distance+1;t++)
	{
		LCD_DrawPoint(uRow,uCol,color);//画点
		xerr+=delta_x;
		yerr+=delta_y;
		if(xerr>distance)
		{
			xerr-=distance;
			uRow+=incx;
		}
		if(yerr>distance)
		{
			yerr-=distance;
			uCol+=incy;
		}
	}
}


/******************************************************************************
      函数说明：画矩形
      入口数据：x1,y1   起始坐标
                x2,y2   终止坐标
                color   矩形的颜色
      返回值：  无
******************************************************************************/
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2,u16 color)
{
	LCD_DrawLine(x1,y1,x2,y1,color);
	LCD_DrawLine(x1,y1,x1,y2,color);
	LCD_DrawLine(x1,y2,x2,y2,color);
	LCD_DrawLine(x2,y1,x2,y2,color);
}


/******************************************************************************
      函数说明：画圆
      入口数据：x0,y0   圆心坐标
                r       半径
                color   圆的颜色
      返回值：  无
******************************************************************************/
void Draw_Circle(u16 x0,u16 y0,u8 r,u16 color)
{
	int a,b;
	a=0;b=r;	  
	while(a<=b)
	{
		LCD_DrawPoint(x0-b,y0-a,color);             //3           
		LCD_DrawPoint(x0+b,y0-a,color);             //0           
		LCD_DrawPoint(x0-a,y0+b,color);             //1                
		LCD_DrawPoint(x0-a,y0-b,color);             //2             
		LCD_DrawPoint(x0+b,y0+a,color);             //4               
		LCD_DrawPoint(x0+a,y0-b,color);             //5
		LCD_DrawPoint(x0+a,y0+b,color);             //6 
		LCD_DrawPoint(x0-b,y0+a,color);             //7
		a++;
		if((a*a+b*b)>(r*r))//判断要画的点是否过远
		{
			b--;
		}
	}
}

/******************************************************************************
      函数说明：显示汉字串
      入口数据：x,y显示坐标
                *s 要显示的汉字串
                fc 字的颜色
                bc 字的背景色
                sizey 字号 可选 16 24 32
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowChinese(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode)
{
	while(*s!=0)
	{
		if(sizey==12) LCD_ShowChinese12x12(x,y,s,fc,bc,sizey,mode);
		else if(sizey==16) LCD_ShowChinese16x16(x,y,s,fc,bc,sizey,mode);
		else if(sizey==24) LCD_ShowChinese24x24(x,y,s,fc,bc,sizey,mode);
		else if(sizey==32) LCD_ShowChinese32x32(x,y,s,fc,bc,sizey,mode);
		else return;
		s+=2;
		x+=sizey;
	}
}

/******************************************************************************
      函数说明：显示单个12x12汉字
      入口数据：x,y显示坐标
                *s 要显示的汉字
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowChinese12x12(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode)
{
	u8 i,j,m=0;
	u16 k;
	u16 HZnum;//汉字数目
	u16 TypefaceNum;//一个字符所占字节大小
	u16 x0=x;
	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	                         
	HZnum=sizeof(tfont12)/sizeof(typFNT_GB12);	//统计汉字数目
	for(k=0;k<HZnum;k++) 
	{
		if((tfont12[k].Index[0]==*(s))&&(tfont12[k].Index[1]==*(s+1)))
		{ 	
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)//非叠加方式
					{
						if(tfont12[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
						else LCD_WR_DATA(bc);
						m++;
						if(m%sizey==0)
						{
							m=0;
							break;
						}
					}
					else//叠加方式
					{
						if(tfont12[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//画一个点
						x++;
						if((x-x0)==sizey)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;  //查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	}
} 

/******************************************************************************
      函数说明：显示单个16x16汉字
      入口数据：x,y显示坐标
                *s 要显示的汉字
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowChinese16x16(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode)
{
	u8 i,j,m=0;
	u16 k;
	u16 HZnum;//汉字数目
	u16 TypefaceNum;//一个字符所占字节大小
	u16 x0=x;
  TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	HZnum=sizeof(tfont16)/sizeof(typFNT_GB16);	//统计汉字数目
	for(k=0;k<HZnum;k++) 
	{
		if ((tfont16[k].Index[0]==*(s))&&(tfont16[k].Index[1]==*(s+1)))
		{ 	
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)//非叠加方式
					{
						if(tfont16[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
						else LCD_WR_DATA(bc);
						m++;
						if(m%sizey==0)
						{
							m=0;
							break;
						}
					}
					else//叠加方式
					{
						if(tfont16[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//画一个点
						x++;
						if((x-x0)==sizey)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;  //查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	}
} 


/******************************************************************************
      函数说明：显示单个24x24汉字
      入口数据：x,y显示坐标
                *s 要显示的汉字
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowChinese24x24(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode)
{
	u8 i,j,m=0;
	u16 k;
	u16 HZnum;//汉字数目
	u16 TypefaceNum;//一个字符所占字节大小
	u16 x0=x;
	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	HZnum=sizeof(tfont24)/sizeof(typFNT_GB24);	//统计汉字数目
	for(k=0;k<HZnum;k++) 
	{
		if ((tfont24[k].Index[0]==*(s))&&(tfont24[k].Index[1]==*(s+1)))
		{ 	
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)//非叠加方式
					{
						if(tfont24[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
						else LCD_WR_DATA(bc);
						m++;
						if(m%sizey==0)
						{
							m=0;
							break;
						}
					}
					else//叠加方式
					{
						if(tfont24[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//画一个点
						x++;
						if((x-x0)==sizey)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;  //查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	}
} 

/******************************************************************************
      函数说明：显示单个32x32汉字
      入口数据：x,y显示坐标
                *s 要显示的汉字
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowChinese32x32(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode)
{
	u8 i,j,m=0;
	u16 k;
	u16 HZnum;//汉字数目
	u16 TypefaceNum;//一个字符所占字节大小
	u16 x0=x;
	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	HZnum=sizeof(tfont32)/sizeof(typFNT_GB32);	//统计汉字数目
	for(k=0;k<HZnum;k++) 
	{
		if ((tfont32[k].Index[0]==*(s))&&(tfont32[k].Index[1]==*(s+1)))
		{ 	
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)//非叠加方式
					{
						if(tfont32[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
						else LCD_WR_DATA(bc);
						m++;
						if(m%sizey==0)
						{
							m=0;
							break;
						}
					}
					else//叠加方式
					{
						if(tfont32[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//画一个点
						x++;
						if((x-x0)==sizey)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;  //查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	}
}


/******************************************************************************
      函数说明：显示单个字符
      入口数据：x,y显示坐标
                num 要显示的字符
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowChar(u16 x,u16 y,u8 num,u16 fc,u16 bc,u8 sizey,u8 mode)
{
	u8 temp,sizex,t,m=0;
	u16 i,TypefaceNum;//一个字符所占字节大小
	u16 x0=x;
	sizex=sizey/2;
	TypefaceNum=(sizex/8+((sizex%8)?1:0))*sizey;
	num=num-' ';    //得到偏移后的值
	LCD_Address_Set(x,y,x+sizex-1,y+sizey-1);  //设置光标位置 
	for(i=0;i<TypefaceNum;i++)
	{ 
		if(sizey==12)temp=ascii_1206[num][i];		       //调用6x12字体
		else if(sizey==16)temp=ascii_1608[num][i];		 //调用8x16字体
		else if(sizey==24)temp=ascii_2412[num][i];		 //调用12x24字体
		else if(sizey==32)temp=ascii_3216[num][i];		 //调用16x32字体
		else return;
		for(t=0;t<8;t++)
		{
			if(!mode)//非叠加模式
			{
				if(temp&(0x01<<t))LCD_WR_DATA(fc);
				else LCD_WR_DATA(bc);
				m++;
				if(m%sizex==0)
				{
					m=0;
					break;
				}
			}
			else//叠加模式
			{
				if(temp&(0x01<<t))LCD_DrawPoint(x,y,fc);//画一个点
				x++;
				if((x-x0)==sizex)
				{
					x=x0;
					y++;
					break;
				}
			}
		}
	}   	 	  
}


/******************************************************************************
      函数说明：显示字符串
      入口数据：x,y显示坐标
                *p 要显示的字符串
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowString(u16 x,u16 y,const u8 *p,u16 fc,u16 bc,u8 sizey,u8 mode)
{         
	while(*p!='\0')
	{       
		LCD_ShowChar(x,y,*p,fc,bc,sizey,mode);
		x+=sizey/2;
		p++;
	}  
}


/******************************************************************************
      函数说明：显示数字
      入口数据：m底数，n指数
      返回值：  无
******************************************************************************/
u32 mypow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;
	return result;
}


/******************************************************************************
      函数说明：显示整数变量
      入口数据：x,y显示坐标
                num 要显示整数变量
                len 要显示的位数
                fc 字的颜色
                bc 字的背景色
                sizey 字号
      返回值：  无
******************************************************************************/
void LCD_ShowIntNum(u16 x,u16 y,u16 num,u8 len,u16 fc,u16 bc,u8 sizey)
{         	
	u8 t,temp;
	u8 enshow=0;
	u8 sizex=sizey/2;
	for(t=0;t<len;t++)
	{
		temp=(num/mypow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				LCD_ShowChar(x+t*sizex,y,' ',fc,bc,sizey,0);
				continue;
			}else enshow=1; 
		 	 
		}
	 	LCD_ShowChar(x+t*sizex,y,temp+48,fc,bc,sizey,0);
	}
} 


/******************************************************************************
      函数说明：显示两位小数变量
      入口数据：x,y显示坐标
                num 要显示小数变量
                len 要显示的位数
                fc 字的颜色
                bc 字的背景色
                sizey 字号
      返回值：  无
******************************************************************************/
void LCD_ShowFloatNum1(u16 x,u16 y,float num,u8 len,u16 fc,u16 bc,u8 sizey)
{         	
	u8 t,temp,sizex;
	u16 num1;
	sizex=sizey/2;
	num1=num*100;
	for(t=0;t<len;t++)
	{
		temp=(num1/mypow(10,len-t-1))%10;
		if(t==(len-2))
		{
			LCD_ShowChar(x+(len-2)*sizex,y,'.',fc,bc,sizey,0);
			t++;
			len+=1;
		}
	 	LCD_ShowChar(x+t*sizex,y,temp+48,fc,bc,sizey,0);
	}
}


/******************************************************************************
      函数说明：显示图片
      入口数据：x,y起点坐标
                length 图片长度
                width  图片宽度
                pic[]  图片数组    
      返回值：  无
******************************************************************************/
void LCD_ShowPicture(u16 x,u16 y,u16 length,u16 width,const u8 pic[])
{
	u16 i,j;
	u32 k=0;
	LCD_Address_Set(x,y,x+length-1,y+width-1);
	for(i=0;i<length;i++)
	{
		for(j=0;j<width;j++)
		{
			LCD_WR_DATA8(pic[k*2]);
			LCD_WR_DATA8(pic[k*2+1]);
			k++;
		}
	}			
}


