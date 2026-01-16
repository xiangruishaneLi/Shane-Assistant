//#include "stm32f10x.h"
//#include <stdio.h>
//#include <string.h>

///**
// * ==============================================================================
// * 项目名称：小瑞助手 (XiaoRui Assistant) 上下位机通信驱动
// * 适配APP版本：v5.2
// * 硬件平台：STM32F103C8T6 + JDY-23 (透传模式)
// * * [实战功能速查]
// * 1. 波形监控 -> 找 XR_Service_SendWave() -> 填入你的目标速度和真实速度
// * 2. 参数调节 -> 找 XR_Service_ParsePID() -> 把收到的数赋值给你的 Kp,Ki,Kd
// * 3. 无线遥控 -> 找 XR_Service_ParseRemote() -> 在 case 里填你的电机控制函数
// * 4. 数据对话 -> 找 XR_Service_ParseChat() -> 在里面写 if(strcmp...) 判断指令
// * 工程潦草如有问题联系邮箱     xiangrui.shane.li@outlook.com
// ©            版权所有 李祥瑞
// ©        本软件仅供学习交流与硬件调试使用
// * ==============================================================================
// */

//// --- 全局变量 (这里放智能车真正的控制变量) ---
//float Real_Kp = 1.5f;
//float Real_Ki = 0.5f;
//float Real_Kd = 0.0f;

//// 速度变量 (用于波形显示)

//int16_t Target_Speed = 500;  // 目标速度 (红线)
//int16_t Measured_Speed = 0;  // 实际速度 (蓝线)
//uint8_t wave_dir = 0;        // 波形方向控制

//// 串口接收缓冲区
//#define RX_BUFFER_SIZE 128
//volatile uint8_t RxBuffer[RX_BUFFER_SIZE];
//volatile uint8_t RxCount = 0;
//volatile uint8_t RxTimeOut = 0;

//// ==============================================================================
////   [模块 1] 底层驱动
//// ==============================================================================

//// 初始化板载 LED (PC13) - 用于指示接收状态
//void XR_LED_Init(void) {
//    GPIO_InitTypeDef GPIO_InitStructure;
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_Init(GPIOC, &GPIO_InitStructure);
//    GPIO_SetBits(GPIOC, GPIO_Pin_13); // 默认灭
//}

//// 初始化串口 (默认 9600)
//void XR_UART_Init(uint32_t baudrate) {
//    GPIO_InitTypeDef GPIO_InitStructure;
//    USART_InitTypeDef USART_InitStructure;
//    NVIC_InitTypeDef NVIC_InitStructure;
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
//    
//    // PA9(TX), PA10(RX)
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_Init(GPIOA, &GPIO_InitStructure);
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//    GPIO_Init(GPIOA, &GPIO_InitStructure);
//    
//    USART_InitStructure.USART_BaudRate = baudrate;
//    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//    USART_InitStructure.USART_StopBits = USART_StopBits_1;
//    USART_InitStructure.USART_Parity = USART_Parity_No;
//    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
//    USART_Init(USART1, &USART_InitStructure);
//    
//    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
//    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);
//    USART_Cmd(USART1, ENABLE);
//}

//void XR_SendByte(uint8_t ch) {
//    USART_SendData(USART1, ch);
//    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
//}

//void XR_SendString(char* str) {
//    while (*str) XR_SendByte(*str++);
//}

//void XR_Delay(uint32_t ms) {
//    uint32_t i, j;
//    for (i = 0; i < ms; i++) {
//        for (j = 0; j < 7200; j++);
//        if (RxTimeOut > 0) RxTimeOut++;
//    }
//}

//// 串口中断服务函数
//void USART1_IRQHandler(void) {
//    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
//        uint8_t res = USART_ReceiveData(USART1);
//        GPIOC->ODR ^= GPIO_Pin_13; // 收到数据翻转LED，调试用
//        if (RxCount < RX_BUFFER_SIZE) RxBuffer[RxCount++] = res;
//        RxTimeOut = 1;
//    }
//}

//// ==============================================================================
////   [模块 2] 核心功能实现 
//// ==============================================================================

///**
// * 【功能 1：波形发送】
// * --------------------------------------------------------
// * [APP界面]: "参数调节" 页面的波形图
// * [实战]: 
// * 在你的定时器中断里(比如每50ms)，或者主循环里调用它。
// * XR_Service_SendWave(期望速度, 编码器测速值);
// * * [参数含义]:
// * target (int16_t): 你的目标值 (红色线)，比如 500
// * actual (int16_t): 你的实际值 (蓝色线)，比如 480
// * --------------------------------------------------------
// */
//void XR_Service_SendWave(int16_t target, int16_t actual) {
//    uint8_t data[7];
//    uint8_t i, sum = 0;
//    
//    // 协议封装 (固定格式 A5 5A ...)
//    data[0] = 0xA5; 
//    data[1] = 0x5A;
//    data[2] = (target >> 8) & 0xFF; 
//    data[3] = target & 0xFF;
//    data[4] = (actual >> 8) & 0xFF; 
//    data[5] = actual & 0xFF;
//    
//    // 计算校验和
//    for (i = 0; i < 6; i++) sum += data[i];
//    data[6] = sum;

//    // 发送出去
//    for (i = 0; i < 7; i++) XR_SendByte(data[i]);
//}

///**
// * 【功能 2：PID 参数解析】
// * --------------------------------------------------------
// * [APP界面]: "参数调节" 页面 -> 输入数字 -> 点击【确认下发】
// * [实战]: 
// * 当单片机收到这三个数时，把它们赋值给你代码里真正的 PID 变量。
// * * [参数含义]:
// * RxBuffer[0]: 对应 APP 上的 "参数 1 (P)"
// * RxBuffer[1]: 对应 APP 上的 "参数 2 (I)"
// * RxBuffer[2]: 对应 APP 上的 "参数 3 (D)"
// * --------------------------------------------------------
// */
//void XR_Service_ParsePID(void) {
//    uint8_t p_raw = RxBuffer[0];
//    uint8_t i_raw = RxBuffer[1];
//    uint8_t d_raw = RxBuffer[2];
//    
//    // --- [用户修改区] ---
//    // 假设你在 APP 填了 15, 5, 10
//    // 这里把它们转成 float 赋值给全局变量
//    Real_Kp = (float)p_raw / 10.0f;  // 15 -> 1.5
//    Real_Ki = (float)i_raw / 10.0f;  // 5  -> 0.5
//    Real_Kd = (float)d_raw;          // 10 -> 10.0
//    
//    // (可选) 打印出来确认一下
//    char msg[64];
//    sprintf(msg, "Set PID OK: P=%.1f I=%.1f D=%.0f\r\n", Real_Kp, Real_Ki, Real_Kd);
//    XR_SendString(msg);
//}

///**
// * 【功能 3：无线遥控】
// * --------------------------------------------------------
// * [APP界面]: "无线遥控" 页面 -> 按下或松开按钮
// * [实战]: 
// * 在 switch-case 里填入你的电机控制函数。
// * * [参数含义 (cmd)]:
// * 0x01: 按下【▲】 -> 车前进
// * 0x02: 按下【▼】 -> 车后退
// * 0x03: 按下【◀】 -> 车左转
// * 0x04: 按下【▶】 -> 车右转
// * 0x00: 松开任意键 -> 车停止 (防撞车)
// * --------------------------------------------------------
// */
//void XR_Service_ParseRemote(void) {
//    uint8_t cmd = RxBuffer[0];
//    
//    switch(cmd) {
//        case 0x01: 
//            // 手机按下了【▲ 前进】
//            // [用户修改区] 填入你的前进代码
//            // Motor_SetSpeed(500, 500); 
//            Target_Speed = 800; // (这里演示用)
//            XR_SendString("CMD: Go\r\n"); 
//            break;
//            
//        case 0x02: 
//            // 手机按下了【▼ 后退】
//            Target_Speed = -800; // (这里演示用)
//            XR_SendString("CMD: Back\r\n"); 
//            break;
//            
//        case 0x03: 
//            // 手机按下了【◀ 左转】
//            XR_SendString("CMD: Left\r\n"); 
//            break;
//            
//        case 0x04: 
//            // 手机按下了【▶ 右转】
//            XR_SendString("CMD: Right\r\n"); 
//            break;
//            
//        case 0x05: // 按钮 A
//            // 可以用来开灯，或者鸣笛
//            XR_SendString("Action A\r\n");
//            break;
//            
//        case 0x06: // 按钮 B
//            XR_SendString("Action B\r\n");
//            break;
//            
//        case 0x00: 
//            // 【重要】手机松开了按钮 -> 停车
//            // [用户修改区] 填入你的停车代码
//            // Motor_SetSpeed(0, 0);
//            Target_Speed = 0; // (这里演示用)
//            XR_SendString("CMD: Stop\r\n"); 
//            break;
//    }
//}

///**
// * 【功能 4：数据对话】
// * --------------------------------------------------------
// * [APP界面]: "数据对话" 页面 -> 发送文本
// * [实战]: 
// * 用来做简单的命令控制，或者打印调试信息。
// * --------------------------------------------------------
// */
//void XR_Service_ParseChat(void) {
//    // 稍微停顿，等待所有字符收完
//    XR_Delay(10);
//    
//    // 把接收缓冲区变成字符串
//    RxBuffer[RxCount] = '\0'; 
//    char* recv_str = (char*)RxBuffer;

//    // --- [用户修改区] 简单的指令解析示例 ---
//    
//    if (strstr(recv_str, "LED_ON")) {
//        // 如果你发了 "LED_ON"
//        GPIO_ResetBits(GPIOC, GPIO_Pin_13); // 点亮 LED
//        XR_SendString("Reply: LED is ON\r\n");
//    }
//    else if (strstr(recv_str, "LED_OFF")) {
//        // 如果你发了 "LED_OFF"
//        GPIO_SetBits(GPIOC, GPIO_Pin_13);   // 熄灭 LED
//        XR_SendString("Reply: LED is OFF\r\n");
//    }
//    else {
//        // 没匹配到指令，默认原样回显
//        XR_SendString("Reply: ");
//        XR_SendString(recv_str);
//    }
//}

//// ==============================================================================
////   主函数 (调度中心)
//// ==============================================================================

//int main(void) {
//    XR_LED_Init();
//    XR_UART_Init(9600);
//    
//    // 上电提示
//    XR_SendString("XiaoRui Assistant Test Mode\r\n");

//    while (1) {
//        // --- 1. 模拟产生三角波 (测试功能1) ---
//        if (wave_dir == 0) {
//            Measured_Speed += 20;
//            if (Measured_Speed >= 1000) wave_dir = 1;
//        } else {
//            Measured_Speed -= 20;
//            if (Measured_Speed <= 0) wave_dir = 0;
//        }
//        // 发送给手机
//        XR_Service_SendWave(Target_Speed, Measured_Speed);


//        // --- 2. 核心：处理手机发来的数据 ---
//        if (RxTimeOut > 2) {
//            // 根据字节长度判断是哪个功能
//            if (RxCount == 3) {
//                // 收到3个字节 -> 测试功能2 (PID)
//                XR_Service_ParsePID();
//            } 
//            else if (RxCount == 1) {
//                // 收到1个字节 -> 测试功能3 (遥控)
//                XR_Service_ParseRemote();
//            } 
//            else {
//                // 其他长度 -> 测试功能4 (聊天)
//                XR_Service_ParseChat();
//            }

//            // 清空状态
//            RxCount = 0;
//            RxTimeOut = 0;
//            
//            // 停顿一下，防止回复的数据被波形数据挤掉
//            XR_Delay(100); 
//        }

//        // 循环延时 (控制波形刷新速度)
//        XR_Delay(50);
//    }
//}


//// ==============================================================================
//// ==============================================================================
//// ==============================================================================
//// ==============================================================================
#include "stm32f10x.h"
#include <stdio.h>
#include <string.h>

/**
 * 小瑞助手 STM32 固件 - 稳定版
 * 确保：
 * 1. 收到数据 LED 闪烁
 * 2. 收到数据必定回复 Reply...
 * 3. 闲时自动发波形
 */

#define RX_BUFFER_SIZE 256
volatile uint8_t RxBuffer[RX_BUFFER_SIZE];
volatile uint8_t RxCount = 0;
volatile uint8_t RxTimeOut = 0;

int16_t target_val = 500;
int16_t actual_val = 0;
uint8_t direction = 0;

void LED_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_SetBits(GPIOC, GPIO_Pin_13); 
}

void USART1_Init(uint32_t baudrate) {
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    USART_Cmd(USART1, ENABLE);
}

void USART_SendByte(uint8_t ch) {
    USART_SendData(USART1, ch);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

void USART_SendString(char* str) {
    while (*str) USART_SendByte(*str++);
}

void Send_Waveform(int16_t target, int16_t actual) {
    uint8_t data[7];
    uint8_t i, sum = 0;
    data[0] = 0xA5; data[1] = 0x5A;
    data[2] = (target >> 8) & 0xFF; data[3] = target & 0xFF;
    data[4] = (actual >> 8) & 0xFF; data[5] = actual & 0xFF;
    for (i = 0; i < 6; i++) sum += data[i];
    data[6] = sum;
    for (i = 0; i < 7; i++) USART_SendByte(data[i]);
}

void Delay_ms(uint32_t ms) {
    uint32_t i, j;
    for (i = 0; i < ms; i++) {
        for (j = 0; j < 7200; j++);
        if (RxTimeOut > 0) RxTimeOut++;
    }
}

void USART1_IRQHandler(void) {
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        uint8_t res = USART_ReceiveData(USART1);
        GPIOC->ODR ^= GPIO_Pin_13; // 翻转LED
        if (RxCount < RX_BUFFER_SIZE) RxBuffer[RxCount++] = res;
        RxTimeOut = 1;
    }
}

int main(void) {
    LED_Init();
    USART1_Init(9600);
    USART_SendString("Ready\r\n");

    while (1) {
        // --- 1. 优先处理回复 ---
        // RxTimeOut > 2 表示已经过了 2ms 没有新数据，认为一包结束
        if (RxTimeOut > 2) {
            Delay_ms(20); // 稍微停一下，让数据稳一稳
            
            USART_SendString("Reply: ");
            for(int k=0; k<RxCount; k++) USART_SendByte(RxBuffer[k]);
            USART_SendString("\r\n"); // 加上换行符

            RxCount = 0; 
            RxTimeOut = 0;
            
            Delay_ms(100); // 回复完多休息一下，避开冲突
        }

        // --- 2. 发送波形 ---
        if (direction == 0) { actual_val += 20; if (actual_val >= 1000) direction = 1; } 
        else { actual_val -= 20; if (actual_val <= 0) direction = 0; }
        
        Send_Waveform(target_val, actual_val);

        Delay_ms(50); // 波形发送间隔
    }
}