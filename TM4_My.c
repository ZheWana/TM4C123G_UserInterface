/**
 * @file TM4_My.c
 * @author ZheWana
 * @brief TM4库函数封装源文件
 * @version 0.1
 * @date 2021-7-9
 * @note The whole file is encoded by UTF-8.
 */

#include "TM4_My.h"

uint32_t Tick = 0;

/**
 * @brief 时钟设置函数
 * @retval void
 */
void SystemClock_Config(void) {
    SysCtlClockSet((SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN));
    SysTickEnable();
    SysTickPeriodSet(SysCtlClockGet() / 1000);//16777216
    SysTickIntEnable();
    SysTickIntRegister(Systick_IntHandler);
    IntPrioritySet(INT_SYSCTL, 0);
}

/**
 * @brief 获取当前Tick值
 * @return uint32_t Tick值
 */
uint32_t Get_Tick(void) {
    return Tick;
}

/**
 * @brief Systick中断处理函数
 */
void Systick_IntHandler(void) {
    Tick++;
}

/**
 * @brief PID实现函数
 * @param ctrl PID控制量句柄
 * @return PID控制量
 */
double PID_Realize(pid *ctrl) {
    ctrl->error.cur = ctrl->ctr.aim - ctrl->ctr.cur;
    ctrl->error.sum += ctrl->error.cur;
    ctrl->error.bia = ctrl->error.cur - ctrl->error.pre;
    ctrl->error.pre = ctrl->error.cur;
    return ctrl->kp * ctrl->error.cur + ctrl->ki * ctrl->error.sum + ctrl->kd * ctrl->error.bia;
}

/**
 * @brief PID初始化函数
 * @param ctrl PID句柄
 * @param kp 比例控制系数
 * @param ki 积分控制系数
 * @param kd 微分控制系数
 * @param aim 目标值
 */
void PID_Init(pid *ctrl, double kp, double ki, double kd, double aim) {
    ctrl->kp = kp;
    ctrl->ki = ki;
    ctrl->kd = kd;
    ctrl->ctr.aim = aim;
}

/**
 * @brief 进行毫秒级延时
 * @param ms 延时的时间值，单位：毫秒
 */
void SysDelay_ms(uint32_t ms) {
    static uint32_t tempTick = 0;
    tempTick = Get_Tick();
    while (Get_Tick() - tempTick < ms);
}

/**
 * @brief 进行微秒级延时
 * @param us 延时的时间值，单位：微秒
 */
void SysDelay_us(uint32_t us) {
    SysCtlDelay(SysCtlClockGet() / 8000000 * us);
}

/**
 * @brief 初始化GPIO的对应引脚类型（输入/输出）
 * @param ui32Port 对应GPIO端口的BASE地址（GPIO_PORTx_BASE）(PortA2G)
 * @param ui8Pins 对应引脚的位包表示(GPIO_PIN_x)
 * @param type 是设置的引脚类型
 * @return 如果成功则返回INIT_OK,
 *         否则返回INIT_ERROR
 */
uint8_t GPIO_Init(uint32_t ui32Port, uint8_t ui8Pins, uint8_t type) {
    uint32_t ui32Peripheral;
    switch (ui32Port) {
        case GPIO_PORTA_BASE:
            ui32Peripheral = SYSCTL_PERIPH_GPIOA;
            break;
        case GPIO_PORTB_BASE:
            ui32Peripheral = SYSCTL_PERIPH_GPIOB;
            break;
        case GPIO_PORTC_BASE:
            ui32Peripheral = SYSCTL_PERIPH_GPIOC;
            break;
        case GPIO_PORTD_BASE:
            ui32Peripheral = SYSCTL_PERIPH_GPIOD;
            break;
        case GPIO_PORTE_BASE:
            ui32Peripheral = SYSCTL_PERIPH_GPIOE;
            break;
        case GPIO_PORTF_BASE:
            ui32Peripheral = SYSCTL_PERIPH_GPIOF;
            break;
        case GPIO_PORTG_AHB_BASE:
            ui32Peripheral = SYSCTL_PERIPH_GPIOG;
            break;
        default:
            return INIT_ERROR;
    }
    SysCtlPeripheralEnable(ui32Peripheral);
    while (!SysCtlPeripheralReady(ui32Peripheral));
    if (type)GPIOPinTypeGPIOOutput(ui32Port, ui8Pins);
    else GPIOPinTypeGPIOInput(ui32Port, ui8Pins);
    return INIT_OK;
}

/**
 * @brief 初始化GPIO的对应引脚的输入中断
 * @note 输入引脚默认上拉，电流负载能力2mA,中断优先级为0x00，如有其他需求，请直接更改函数源码
 * @param ui32Port 中断GPIO端口的BASE地址（GPIO_PORTx_BASE）(PortA2G)
 * @param ui8Pins 中断引脚的位包表示(GPIO_PIN_x)
 * @param ui32IntType 指定中断触发机制类型(参数范围如下)
 * @param pfnIntHandler 中断回调函数名
 *        GPIO_FALLING_EDGE/GPIO_RISING_EDGE/GPIO_BOTH_EDGES/
 *        GPIO_LOW_LEVEL/GPIO_HIGH_LEVEL
 * @return 如果成功则返回INIT_OK,
 *         否则返回INIT_ERROR
 */
uint8_t GPIO_INTInit(uint32_t ui32Port, uint8_t ui8Pins, uint32_t ui32IntType, void (*pfnIntHandler)(void)) {
    uint32_t ui32Int;
    if (GPIO_Init(ui32Port, ui8Pins, GPIO_Input) == INIT_ERROR)return INIT_ERROR;

    //GPIO端口引脚解锁
    if (HWREG(ui32Port + GPIO_O_LOCK) != GPIO_LOCK_UNLOCKED) {
        HWREG(ui32Port + GPIO_O_LOCK) = GPIO_LOCK_KEY;
        HWREG(ui32Port + GPIO_O_CR) |= ui8Pins;
        HWREG(ui32Port + GPIO_O_LOCK) = 0x0;
    }

    //默认引脚上拉
    GPIOPadConfigSet(ui32Port, ui8Pins, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    GPIOIntTypeSet(ui32Port, ui8Pins, ui32IntType);
    GPIOIntEnable(ui32Port, ui8Pins);
    GPIOIntRegister(ui32Port, pfnIntHandler);
    switch (ui32Port) {
        case GPIO_PORTA_BASE:
            ui32Int = INT_GPIOA;
            break;
        case GPIO_PORTB_BASE:
            ui32Int = INT_GPIOB;
            break;
        case GPIO_PORTC_BASE:
            ui32Int = INT_GPIOC;
            break;
        case GPIO_PORTD_BASE:
            ui32Int = INT_GPIOD;
            break;
        case GPIO_PORTE_BASE:
            ui32Int = INT_GPIOE;
            break;
        case GPIO_PORTF_BASE:
            ui32Int = INT_GPIOF;
            break;
        case GPIO_PORTG_BASE:
            ui32Int = INT_GPIOG;
            break;
        default:
            return INIT_ERROR;
    }
    IntEnable(ui32Int);
    IntMasterEnable();
    IntPrioritySet(ui32Int, 0x00);
    return INIT_OK;
}

/**
 * @brief 对GPIO引脚执行写操作
 * @param ui32Port 对应GPIO端口的BASE地址（GPIO_PORTx_BASE）
 * @param ui8Pins 对应引脚的位包表示(GPIO_PIN_x)
 * @param state 引脚状态（置高/置低->1/0）
 */
void GPIO_WriteState(uint32_t ui32Port, uint8_t ui8Pins, uint8_t state) {
    GPIOPinWrite(ui32Port, ui8Pins, state ? ui8Pins : 0);
}

/**
 * @brief 对GPIO引脚执行读操作
 * @param ui32Port 对应GPIO端口的BASE地址（GPIO_PORTx_BASE）
 * @param ui8Pins 对应引脚的位包表示(GPIO_PIN_x)
 * @return 引脚状态（置高/置低->1/0）
 */
uint8_t GPIO_ReadState(uint32_t ui32Port, uint8_t ui8Pins) {
    return GPIOPinRead(ui32Port, ui8Pins) == ui8Pins;
}

/**
 * @brief 初始化PWM输出
 * @note ui32PWMPin定义的数据帧类型及大小为uint32
 *       数据帧格式：
 *       低位到高位依次：0:5 十进制两位数：个位为Out序号，十位为Gen序号
 *                     6:9 0x0/0x4/0x8/0xc
 *                     10:17 GPIO_PIN_x
 *                     18 SYSCTL_PERIPH_PWMx(0/1)
 *                     19:30 SYSCTL_PERIPH_GPIOx(A2F)
 *                     31 Reserved
 * @param ui32ClkDiv PWM时钟分频（SYSCTL_PWMDIV_xx）
 * @param ui32PWMPin PWM对应引脚(PWM_PIN_xx_Mx)
 * @param ui32PWMOut 传入变量，返回PWM参数调节所需的PWM输出参数（PWM_OUT_x）
 * @param ui32Gen 传入变量，返回PWM参数调节所需的PWM生成器参数（PWM_Gen_x）
 * @return 初始化成功则返回对应PWM模块地址，
 *         否则返回INIT_ERROR
 */
uint32_t PWM_Init(uint32_t ui32ClkDiv, uint32_t ui32PWMPin,
                  uint32_t *ui32PWMOut, uint32_t *ui32Gen) {
    uint8_t ui8Pins = (ui32PWMPin << 14) >> 24;
    uint32_t ui32Port, ui32PinConfig, pwmbase = ui32PWMPin & 1 << 18;
    SysCtlPWMClockSet(ui32ClkDiv);

    SysCtlPeripheralEnable(pwmbase ? SYSCTL_PERIPH_PWM1 : SYSCTL_PERIPH_PWM0);
    while (!SysCtlPeripheralReady(pwmbase ? SYSCTL_PERIPH_PWM1 : SYSCTL_PERIPH_PWM0));
    SysCtlPeripheralEnable(ui32PWMPin >> 19 | 0xf0000000);
    while (!SysCtlPeripheralReady(ui32PWMPin >> 19 | 0xf0000000));

    switch (ui32PWMPin >> 19 | 0xf0000000) {
        case SYSCTL_PERIPH_GPIOA:
            ui32Port = GPIO_PORTA_BASE;
            ui32PinConfig = 0x01 << 12;
            break;
        case SYSCTL_PERIPH_GPIOB:
            ui32Port = GPIO_PORTB_BASE;
            ui32PinConfig = 0x11 << 12;
            break;
        case SYSCTL_PERIPH_GPIOC:
            ui32Port = GPIO_PORTC_BASE;
            ui32PinConfig = 0x21 << 12;
            break;
        case SYSCTL_PERIPH_GPIOD:
            ui32Port = GPIO_PORTD_BASE;
            ui32PinConfig = 0x30 << 12;
            break;
        case SYSCTL_PERIPH_GPIOE:
            ui32Port = GPIO_PORTE_BASE;
            ui32PinConfig = 0x41 << 12;
            break;
        case SYSCTL_PERIPH_GPIOF:
            ui32Port = GPIO_PORTF_BASE;
            ui32PinConfig = 0x50 << 12;
            break;
        default:
            ui32Port = INIT_ERROR;
            ui32PinConfig = INIT_ERROR;
            return INIT_ERROR;
    }
    GPIOPinTypePWM(ui32Port, ui8Pins);
    GPIOPinConfigure(ui32PinConfig | (ui32PWMPin & 0x3c0 << 2) | ((!(pwmbase == 0)) + 4));

    //GPIO端口引脚解锁
    if (HWREG(ui32Port + GPIO_O_LOCK) != GPIO_LOCK_UNLOCKED) {
        HWREG(ui32Port + GPIO_O_LOCK) = GPIO_LOCK_KEY;
        HWREG(ui32Port + GPIO_O_CR) |= ui8Pins;
        HWREG(ui32Port + GPIO_O_LOCK) = 0x0;
    }

    switch ((ui32PWMPin & 0x3f) / 10) {
        case 1:
            *ui32Gen = PWM_GEN_1;
            break;
        case 2:
            *ui32Gen = PWM_GEN_2;
            break;
        case 3:
            *ui32Gen = PWM_GEN_3;
            break;
        case 0:
            *ui32Gen = PWM_GEN_0;
            break;
        default:
            *ui32Gen = INIT_ERROR;
            return INIT_ERROR;

    }
    PWMGenConfigure(pwmbase ? PWM1_BASE : PWM0_BASE,
                    *ui32Gen,
                    PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC | PWM_GEN_MODE_DBG_RUN | PWM_GEN_MODE_GEN_NO_SYNC |
                    PWM_GEN_MODE_DB_NO_SYNC);
    PWMOutputState(pwmbase ? PWM1_BASE : PWM0_BASE, 1 << ((ui32PWMPin & 0xff)) % 10, true);
    PWMGenEnable(pwmbase ? PWM1_BASE : PWM0_BASE, *ui32Gen);

    switch (((ui32PWMPin & 0x3f)) % 10) {
        case 0:
            *ui32PWMOut = PWM_OUT_0;
            break;
        case 1:
            *ui32PWMOut = PWM_OUT_1;
            break;
        case 2:
            *ui32PWMOut = PWM_OUT_2;
            break;
        case 3:
            *ui32PWMOut = PWM_OUT_3;
            break;
        case 4:
            *ui32PWMOut = PWM_OUT_4;
            break;
        case 5:
            *ui32PWMOut = PWM_OUT_5;
            break;
        case 6:
            *ui32PWMOut = PWM_OUT_6;
            break;
        case 7:
            *ui32PWMOut = PWM_OUT_7;
            break;
        default:
            *ui32PWMOut = INIT_ERROR;
            return INIT_ERROR;
    }
    return pwmbase ? PWM1_BASE : PWM0_BASE;
}

/**
 * @brief PWM频率设置
 * @note 由于PWM时钟分频限制，因此输出特定频率的PWM波需要自行计算时钟分频系数，
 *       具体表现为：低时钟分频无高频限制，有低频限制；
 *                 高时钟分频无低频限制，有高频限制。
 * @param ui32Base PWM模块的地址（PWMx_BASE）
 * @param ui32Gen PWM参数调节所需的PWM生成器参数（PWM_Gen_x）
 * @param NumofDiv PWM时钟相对系统主频的分频数（eg.32分频->32）
 * @param Freq PWM频率
 */
void PWM_FreqSet(uint32_t ui32Base, uint32_t ui32Gen, uint8_t NumofDiv, uint32_t Freq) {
    PWMGenPeriodSet(ui32Base, ui32Gen, SysCtlClockGet() / NumofDiv / Freq);
}

/**
 * @brief PWM占空比设置
 * @param ui32Base PWM模块的地址（PWMx_BASE）
 * @param ui32Gen PWM参数调节所需的PWM生成器参数（PWM_Gen_x）
 * @param ui32PWMOut PWM参数调节所需的PWM输出参数（PWM_OUT_x）
 * @param duty PWM占空比值（0~100）
 */
void PWM_DutySet(uint32_t ui32Base, uint32_t ui32Gen, uint32_t ui32PWMOut, uint8_t duty) {
    PWMPulseWidthSet(ui32Base, ui32PWMOut, PWMGenPeriodGet(ui32Base, ui32Gen) * duty / 100);
}

/**
 * @brief 串口格式化输出
 * @note 调用的串口输出函数为：UART_printf（）
 *       默认输出引脚为A0和A1（连接着launchpad调试器）
 */
void UART_RetargetInit(void) {
    UART_Init(UART0A);
}

/**
 * @brief 串口重定向
 * @param fmt 格式化内容
 * @param ... 参数列表
 * @return 字符个数
 */
int UART_printf(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    int length;
    char buffer[256];
    length = vsnprintf(buffer, 256, fmt, ap);
    for (int i = 0; i < length; i++)
        UARTCharPut(UART0_BASE, buffer[i]);
    va_end(ap);
    return length;
}

/**
 * @brief 串口初始化函数
 * @note 默认波特率115200，
 *       停止位：1
 *       奇偶校验：无
 *       如有其他需求，请更改源码：）
 * @param uart_channel 串口频道编号（UARTxx）
 * @return 初始化成功则返回串口基地址,
 *         否则返回INIT_ERROR
 */
uint32_t UART_Init(uint32_t uart_channel) {
    uint32_t tx = UART_Pinconfig[uart_channel],
            ui32Peripheral,
            ui32Port,
            ui8Pins,
            ui32Base = UART0_BASE | (uart_channel << 12);
    SysCtlPeripheralEnable(uart_channel | 0xf0001800);
    switch (tx >> 12) {
        case 0x00:
            ui32Port = GPIO_PORTA_BASE;
            ui32Peripheral = SYSCTL_PERIPH_GPIOA;
            ui8Pins = GPIO_PIN_0;
            break;
        case 0x10:
            ui32Port = GPIO_PORTB_BASE;
            ui32Peripheral = SYSCTL_PERIPH_GPIOB;
            ui8Pins = GPIO_PIN_0;
            break;
        case 0x21:
            ui32Port = GPIO_PORTC_BASE;
            ui32Peripheral = SYSCTL_PERIPH_GPIOC;
            ui8Pins = (uart_channel - 3) ? GPIO_PIN_4 : GPIO_PIN_6;
            break;
        case 0x31:
            ui32Port = GPIO_PORTD_BASE;
            ui32Peripheral = SYSCTL_PERIPH_GPIOD;
            ui8Pins = (uart_channel - 2) ? GPIO_PIN_4 : GPIO_PIN_6;
            break;
        case 0x40:
            ui32Port = GPIO_PORTE_BASE;
            ui32Peripheral = SYSCTL_PERIPH_GPIOE;
            ui8Pins = (uart_channel - 5) ? GPIO_PIN_4 : GPIO_PIN_0;
            break;
        case 0x41:
            ui32Port = GPIO_PORTE_BASE;
            ui32Peripheral = SYSCTL_PERIPH_GPIOE;
            ui8Pins = (uart_channel - 5) ? GPIO_PIN_4 : GPIO_PIN_0;
            break;
        default:
            return INIT_ERROR;
    }
    SysCtlPeripheralEnable(ui32Peripheral);
    GPIOPinConfigure(tx);
    GPIOPinConfigure(tx | 0x400);
    GPIOPinTypeUART(ui32Port, ui8Pins | ui8Pins << 1);
    UARTConfigSetExpClk(ui32Base,
                        SysCtlClockGet(),
                        115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    return ui32Base;
}

/**
 * @brief 串口中断初始化函数
 * @note 调用此函数则无需调用串口初始化函数
 *       默认: 波特率115200，
 *            停止位：1
 *            奇偶校验：无
 *            如有其他需求，请更改源码：）
 * @param uart_channel 串口频道编号（UARTxx）
 * @param pfnHandler 中断回调函数名
 * @return 初始化成功则返回INIT_OK,
 *         否则返回INIT_ERROR
 */
uint8_t UART_RxINTInit(uint32_t uart_channel, void (*pfnHandler)(void)) {
    uint32_t ui32Base = UART_Init(uart_channel),
            ui32Int;
    if (ui32Base == INIT_ERROR)return INIT_ERROR;
    switch (ui32Base) {
        case UART0_BASE:
            ui32Int = INT_UART0;
            break;
        case UART1_BASE:
            ui32Int = INT_UART1;
            break;
        case UART2_BASE:
            ui32Int = INT_UART2;
            break;
        case UART3_BASE:
            ui32Int = INT_UART3;
            break;
        case UART4_BASE:
            ui32Int = INT_UART4;
            break;
        case UART5_BASE:
            ui32Int = INT_UART5;
            break;
        case UART6_BASE:
            ui32Int = INT_UART6;
            break;
        case UART7_BASE:
            ui32Int = INT_UART7;
            break;
        default:
            return INIT_ERROR;
    }
    UARTIntRegister(ui32Base, pfnHandler);
    IntPrioritySet(ui32Int, 0x00);
    UARTIntEnable(ui32Base, UART_INT_RX | UART_INT_RT);
    IntMasterEnable();
    return INIT_OK;
}

/**
 * @brief 定时器中断初始化函数
 * @param ui32Interrupt 指定中断（INT_TIMERxx）
 * @param ui32Freq 中断触发的频率
 * @param pfnHandler 中断回调函数名
 * @return 初始化成功则返回INIT_OK,
 *         否则返回INIT_ERROR
 */
uint8_t Timer_INTInit(uint32_t ui32Interrupt, uint32_t ui32Freq, void (*pfnHandler)(void)) {
    uint32_t ui32Base, ui32Timer, ui32Peripheral;
    switch (ui32Interrupt) {
        case INT_TIMER0A:
            ui32Peripheral = SYSCTL_PERIPH_TIMER0;
            ui32Base = TIMER0_BASE;
            ui32Timer = TIMER_A;
            break;
        case INT_TIMER0B:
            ui32Peripheral = SYSCTL_PERIPH_TIMER0;
            ui32Base = TIMER0_BASE;
            ui32Timer = TIMER_B;
            break;
        case INT_TIMER1A:
            ui32Peripheral = SYSCTL_PERIPH_TIMER1;
            ui32Base = TIMER1_BASE;
            ui32Timer = TIMER_A;
            break;
        case INT_TIMER1B:
            ui32Peripheral = SYSCTL_PERIPH_TIMER1;
            ui32Base = TIMER1_BASE;
            ui32Timer = TIMER_B;
            break;
        case INT_TIMER2A:
            ui32Peripheral = SYSCTL_PERIPH_TIMER2;
            ui32Base = TIMER2_BASE;
            ui32Timer = TIMER_A;
            break;
        case INT_TIMER2B:
            ui32Peripheral = SYSCTL_PERIPH_TIMER2;
            ui32Base = TIMER2_BASE;
            ui32Timer = TIMER_B;
            break;
        case INT_TIMER3A:
            ui32Peripheral = SYSCTL_PERIPH_TIMER3;
            ui32Base = TIMER3_BASE;
            ui32Timer = TIMER_A;
            break;
        case INT_TIMER3B:
            ui32Peripheral = SYSCTL_PERIPH_TIMER3;
            ui32Base = TIMER3_BASE;
            ui32Timer = TIMER_B;
            break;
        case INT_TIMER4A:
            ui32Peripheral = SYSCTL_PERIPH_TIMER4;
            ui32Base = TIMER4_BASE;
            ui32Timer = TIMER_A;
            break;
        case INT_TIMER4B:
            ui32Peripheral = SYSCTL_PERIPH_TIMER4;
            ui32Base = TIMER4_BASE;
            ui32Timer = TIMER_B;
            break;
        case INT_TIMER5A:
            ui32Peripheral = SYSCTL_PERIPH_TIMER5;
            ui32Base = TIMER5_BASE;
            ui32Timer = TIMER_A;
            break;
        case INT_TIMER5B:
            ui32Peripheral = SYSCTL_PERIPH_TIMER5;
            ui32Base = TIMER5_BASE;
            ui32Timer = TIMER_B;
            break;
        default:
            return INIT_ERROR;
    }
    IntPrioritySet(ui32Interrupt, 0x00);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(ui32Base, TIMER_CFG_PERIODIC);
    TimerLoadSet(ui32Base, ui32Timer, SysCtlClockGet() / ui32Freq - 1);
    TimerIntRegister(ui32Base, ui32Timer, pfnHandler);
    IntEnable(ui32Interrupt);
    TimerIntEnable(ui32Base, TIMER_TIMA_TIMEOUT);
    IntMasterEnable();
    TimerEnable(ui32Base, ui32Timer);
    return INIT_OK;
}

uint32_t Encoder1INTPortB,
        Encoder1INTPinB,
        Encoder1INTPortA,
        Encoder1INTPinA,
        Encoder2INTPortB,
        Encoder2INTPinB,
        Encoder2INTPortA,
        Encoder2INTPinA;
int32_t CNT1 = 0,
        CNT2 = 0;

/**
 * @brief 软件编码器模式中断初始化函数
 * @param ui32PortA 编码器A相通道
 * @param ui8PinA 编码器A相引脚号
 * @param ui32PortB 编码器B相通道
 * @param ui8PinB 编码器B相引脚号
 * @return 初始化成功则返回INIT_OK,
 *         否则返回INIT_ERROR
 */
uint8_t Encoder1_INTInit(uint32_t ui32PortA, uint8_t ui8PinA, uint32_t ui32PortB, uint8_t ui8PinB) {
    if (GPIO_Init(ui32PortA, ui8PinA, GPIO_Input) == INIT_ERROR)
        return INIT_ERROR;
    if (GPIO_INTInit(ui32PortB, ui8PinB, GPIO_RISING_EDGE, Encoder_IRQHandler) == INIT_ERROR)
        return INIT_ERROR;
    Encoder1INTPortA = ui32PortA;
    Encoder1INTPinA = ui8PinA;
    Encoder1INTPortB = ui32PortB;
    Encoder1INTPinB = ui8PinB;
    return INIT_OK;
}

/**
 * @brief 软件编码器模式中断初始化函数
 * @param ui32PortA 编码器A相通道
 * @param ui8PinA 编码器A相引脚号
 * @param ui32PortB 编码器B相通道
 * @param ui8PinB 编码器B相引脚号
 * @return 初始化成功则返回INIT_OK,
 *         否则返回INIT_ERROR
 */
uint8_t Encoder2_INTInit(uint32_t ui32PortA, uint8_t ui8PinA, uint32_t ui32PortB, uint8_t ui8PinB) {
    if (GPIO_Init(ui32PortA, ui8PinA, GPIO_Input) == INIT_ERROR)
        return INIT_ERROR;
    if (GPIO_INTInit(ui32PortB, ui8PinB, GPIO_RISING_EDGE, Encoder_IRQHandler) == INIT_ERROR)
        return INIT_ERROR;
    Encoder2INTPortA = ui32PortA;
    Encoder2INTPinA = ui8PinA;
    Encoder2INTPortB = ui32PortB;
    Encoder2INTPinB = ui8PinB;
    return INIT_OK;
}

/**
 * @brief 编码器中断回调函数
 */
void Encoder_IRQHandler(void) {
    uint32_t status1 = GPIOIntStatus(Encoder1INTPortB, true),
            status2 = GPIOIntStatus(Encoder2INTPortB, true);
    GPIOIntClear(Encoder1INTPortB, status1);
    GPIOIntClear(Encoder2INTPortB, status2);
    if ((status1 & Encoder1INTPinB) == Encoder1INTPinB) {
        GPIO_ReadState(Encoder1INTPortA, Encoder1INTPinA) ? CNT1++ : CNT1--;
    }
    if ((status2 & Encoder2INTPinB) == Encoder2INTPinB) {
        GPIO_ReadState(Encoder2INTPortA, Encoder2INTPinA) ? CNT2++ : CNT2--;
    }
}

#ifdef MPU6050

float pitch = 0, roll = 0, yaw = 0;
static signed char gyro_orientation[9] = {1, 0, 0,
                                          0, 1, 0,
                                          0, 0, 1};

uint8_t mpu_dmp_get_data(float *pitch, float *roll, float *yaw) {
    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    unsigned long sensor_timestamp;
    short gyro[3], accel[3], sensors;
    unsigned char more;
    long quat[4];
    if (dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more))return 1;
    if (sensors & INV_WXYZ_QUAT) {
        q0 = quat[0] / q30;
        q1 = quat[1] / q30;
        q2 = quat[2] / q30;
        q3 = quat[3] / q30;
        *pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;
        *roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3;
        *yaw = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3;
    } else return 2;
    return 0;
}


int run_self_test(void) {
    int result;
    char test_packet[4] = {0};
    long gyro[3], accel[3];
    result = mpu_run_self_test(gyro, accel);
    if (result == 0x7) {
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long) (gyro[0] * sens);
        gyro[1] = (long) (gyro[1] * sens);
        gyro[2] = (long) (gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
        return 0;
    } else return 1;
}

uint8_t mpu_dmp_init(void) {
    uint8_t res = 0;
    struct int_param_s int_param;
    if (mpu_init(&int_param) == 0)
    {
        res = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
        if (res)return 1;
        res = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
        if (res)return 2;
        res = mpu_set_sample_rate(100);
        if (res)return 3;
        res = dmp_load_motion_driver_firmware();
        if (res)return 4;
        res = dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
                                 DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
                                 DMP_FEATURE_GYRO_CAL);
        if (res)return 6;
        res = dmp_set_fifo_rate(100);
        if (res)return 7;
        res = run_self_test();
        if (res)return 8;
        res = mpu_set_dmp_state(1);
        if (res)return 9;
    } else return 10;
    return 0;
}

#endif//MPU6050

#ifdef U8g2

u8g2_t u8g2;

void SPI_Transmit(uint32_t ui32Base, uint8_t *pData, uint8_t length) {
    int i = 0;
    for (i = 0; i < length; i++) {
        SSIDataGet(ui32Base, (uint32_t*)pData);
    }
}

uint8_t u8x8_byte_4wire_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    switch (msg) {
        case U8X8_MSG_BYTE_SEND:
            SPI_Transmit(SSI0_BASE, (uint8_t *) arg_ptr, arg_int);
            break;
        case U8X8_MSG_BYTE_INIT:
            break;
        case U8X8_MSG_BYTE_SET_DC:
            GPIO_WriteState(OLED_DC_GPIO_Port, OLED_DC_Pin, arg_int);
            break;
        case U8X8_MSG_BYTE_START_TRANSFER:
            GPIO_WriteState(OLED_NSS_GPIO_Port, OLED_NSS_Pin, 0);
            break;
        case U8X8_MSG_BYTE_END_TRANSFER:
            GPIO_WriteState(OLED_NSS_GPIO_Port, OLED_NSS_Pin, 1);
            break;
        default:
            return 0;
    }
    return 1;
}

uint8_t u8x8_gpio_and_delay(U8X8_UNUSED u8x8_t *u8x8, U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int,
                            U8X8_UNUSED void *arg_ptr) {
    switch (msg) {
        case U8X8_MSG_GPIO_AND_DELAY_INIT:
            Delay(1);
            break;
        case U8X8_MSG_DELAY_MILLI:
            Delay(arg_int);
            break;
        case U8X8_MSG_GPIO_DC:
            GPIO_WriteState(OLED_DC_GPIO_Port, OLED_DC_Pin, arg_int);
            break;
        case U8X8_MSG_GPIO_RESET:
            GPIO_WriteState(OLED_RES_GPIO_Port, OLED_RES_Pin, arg_int);
            break;
    }
    return 1;
}

#endif//U8g2

#ifdef UART_INT
char data, charflag;
int negtiveflag, pointflag;
double valint, valpoint;

/**
 * @brief 串口中断回调函数
 */
void UART_IRQHandler(void) {
    UARTIntClear(UART0_BASE, INT_UART0);
    while (UARTCharsAvail(UART0_BASE)) {
        data = UARTCharGet(UART0_BASE);
        if (data == '\n' || data == 'x' || data == 'y') {
            while (valpoint >= 1)valpoint /= 10;
            switch (charflag) {
                case 'x':
                    x = negtiveflag ? (-1) * (valint + valpoint) : (valint + valpoint);
                    charflag=0;
                    break;
                case 'y':
                    y = negtiveflag ? (-1) * (valint + valpoint) : (valint + valpoint);
                    charflag=0;
                    break;
                default:;
            }
        }
        if (data == 'x') {
            charflag = 'x';
            negtiveflag = 0;
            pointflag = 0;
            valint = 0;
            valpoint = 0;
            break;
        }
        if (data == 'y') {
            charflag = 'y';
            negtiveflag = 0;
            pointflag = 0;
            valint = 0;
            valpoint = 0;
            break;
        }
        if (data == '-')negtiveflag = 1;
        if (data == '.')pointflag = 1;
        if (data <= '9' && data >= '0') {
            if (pointflag) {
                valpoint = valpoint * 10 + (data - '0') * 1;
            } else {
                valint = valint * 10 + (data - '0') * 1;
            }
        }
    }
}
#endif

//void GPIOF0_IRQHandler(void) {
//    uint32_t intstatus = GPIOIntStatus(GPIO_PORTF_BASE, true);
//    GPIOIntClear(GPIO_PORTF_BASE, intstatus);
//    if ((intstatus & GPIO_PIN_4) == GPIO_PIN_4) {
//        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 1 << 1);
//    }
//}