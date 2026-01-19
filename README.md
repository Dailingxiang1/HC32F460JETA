# HC32F460JETA 外设驱动集合

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![HC32](https://img.shields.io/badge/HC32-HC32F460JETA-green)](https://www.hdsc.com.cn/)
[![LVGL](https://img.shields.io/badge/LVGL-8.3-blue)](https://lvgl.io/)

&gt; HC32F460JETA 微控制器的多功能外设驱动集合，支持 QSPI Flash、LVGL 显示、传感器和低功耗模式。

---

## 📋 目录
- [项目简介](#项目简介)
- [功能模块](#功能模块)
- [快速开始](#快速开始)
- [硬件要求](#硬件要求)
- [配置指南](#配置指南)
- [使用示例](#使用示例)
- [注意事项](#注意事项)
- [技术支持](#技术支持)

---

## 🎯 项目简介

本项目为HC32F460JETA微控制器提供了一套完整的外设驱动解决方案，集成了显示、存储、传感器、低功耗管理等常用功能，适用于物联网、智能硬件等嵌入式应用场景。

---

## 🔧 功能模块

### 🚀 存储系统
#### QSPI Flash 驱动
- **内存映射功能** - 外部Flash直接映射到MCU内存空间
- **高速数据传输** - 支持四线SPI模式
- **标准兼容性** - 默认适配华邦W25Q64系列
``` bash
# QSPI初始化
void SystemInit_QspiMem(void)
{
    /* QSPI configure */
    CM_GPIO->PWPR = 0xA501U;
    /* High driver */
    CM_GPIO->PCRB1  = 0x0020U;
    CM_GPIO->PCRB14  = 0x0020U;
    CM_GPIO->PCRB13  = 0x0020U;
    CM_GPIO->PCRB12  = 0x0020U;
    CM_GPIO->PCRB10 = 0x0020U;
    CM_GPIO->PCRB2 = 0x0020U;
    /* Set function */
    CM_GPIO->PFSRB1  = 0x07U;
    CM_GPIO->PFSRB14  = 0x07U;
    CM_GPIO->PFSRB13  = 0x07U;
    CM_GPIO->PFSRB12  = 0x07U;
    CM_GPIO->PFSRB10 = 0x07U;
    CM_GPIO->PFSRB2 = 0x07U;
    /* qspi configure */
    CM_PWC->FCG1 &= ~0x00000008UL;
	CM_QSPI->CR   = 0x0001000D;
    CM_QSPI->CSCR = 0x00000011;
    CM_QSPI->FCR  = 0x00008332;
}
# 分散加载文件的配置
EX_QSPI_ROM 0x98000000 0x800000  {
    QSPI_ROM 0x98000000 0x800000  {
        *.o (.ex_rom)
		lv_font*.o (+RO)
    }
}
```
### 🖥️ 显示系统
#### LVGL 8.3 图形界面
- **SPI接口驱动** - 高效LCD显示屏控制
- **丰富UI组件** - 按钮、标签、列表等完整控件库
- **触摸支持** - 可扩展触摸功能

### 📡 传感器系统
#### DA213B 三轴加速度传感器
- **I2C通信接口** - 标准I2C协议，易于集成
- **高精度测量** - 三轴加速度数据采集
- **低功耗设计** - 适合电池供电应用

### 💡 执行器系统
#### LED控制
- **PWM调光** - 256级亮度调节
- **呼吸灯效果** - 支持渐变亮度变化

#### 震动马达
- **强度可调** - 多级震动强度控制
- **模式定制** - 支持不同震动模式

### ⏰ 时钟系统
#### RTC实时时钟
- **日历功能** - 年月日时分秒完整计时
- **闹钟功能** - 支持定时唤醒
- **低功耗运行** - 休眠模式下持续工作

### 🔋 电源管理
#### 低功耗模式
- **多种休眠等级** - 睡眠、停机、待机模式
- **快速唤醒** - 多源唤醒机制
- **功耗优化** - 智能外设电源管理

### ⚙️ 系统服务
#### AOS运行系统
- **事件驱动** - 软件触发绑定机制
- **任务调度** - 轻量级任务管理
- **模块化设计** - 易于扩展和维护

---

## 🚀 快速开始

### 环境要求
- Keil MDK 5.0 或更高版本
- HC32F460 支持包
- JLINK 调试器

### 编译步骤
```bash
# 克隆项目
git clone https://github.com/Dailingxiang1/HC32F460JETA.git

# 打开工程
# 使用Keil打开 project/HC32F460JETA.uvprojx

# 编译下载
# 点击编译按钮，通过JLINK下载到目标板