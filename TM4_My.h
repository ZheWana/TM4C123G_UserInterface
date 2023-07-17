/**
 * @file TM4_My.h
 * @author ZheWana
 * @brief TM4库函数封装头文件
 * @version 0.1
 * @date 2021-7-9
 * @note The whole file is encoded by UTF-8.
 */

#ifndef TM4TEMPLATE_TM4_MY_H
#define TM4TEMPLATE_TM4_MY_H

#include "stdio.h"
#include "stdint.h"
#include "stdarg.h"
#include "stdbool.h"
#include "inc/headfile.h"

#define GPIO_Output                          1
#define GPIO_Input                           0
#define INIT_ERROR                           0xff
#define INIT_OK                              1
#define log                                  UART_printf
#define UNUSE                                __attribute__((unused))

enum PWM_PIN_M {
    PWM_PIN_B4_M0_G0 = SYSCTL_PERIPH_GPIOB << 19 | 0 << 18 | GPIO_PIN_4 << 10 | 0x0 << 6 | 12,
    PWM_PIN_B5_M0_G0 = SYSCTL_PERIPH_GPIOB << 19 | 0 << 18 | GPIO_PIN_5 << 10 | 0x4 << 6 | 13,
    PWM_PIN_B6_M0_G1 = SYSCTL_PERIPH_GPIOB << 19 | 0 << 18 | GPIO_PIN_6 << 10 | 0x8 << 6 | 0,
    PWM_PIN_B7_M0_G1 = SYSCTL_PERIPH_GPIOB << 19 | 0 << 18 | GPIO_PIN_7 << 10 | 0xc << 6 | 1,
    PWM_PIN_C4_M0_G2 = SYSCTL_PERIPH_GPIOC << 19 | 0 << 18 | GPIO_PIN_4 << 10 | 0x0 << 6 | 36,
    PWM_PIN_C5_M0_G2 = SYSCTL_PERIPH_GPIOC << 19 | 0 << 18 | GPIO_PIN_5 << 10 | 0x4 << 6 | 37,
    PWM_PIN_D0_M0_G3 = SYSCTL_PERIPH_GPIOD << 19 | 0 << 18 | GPIO_PIN_0 << 10 | 0x0 << 6 | 36,
    PWM_PIN_D1_M0_G3 = SYSCTL_PERIPH_GPIOD << 19 | 0 << 18 | GPIO_PIN_1 << 10 | 0x4 << 6 | 37,
    PWM_PIN_E4_M0_G4 = SYSCTL_PERIPH_GPIOE << 19 | 0 << 18 | GPIO_PIN_4 << 10 | 0x0 << 6 | 24,
    PWM_PIN_E5_M0_G4 = SYSCTL_PERIPH_GPIOE << 19 | 0 << 18 | GPIO_PIN_5 << 10 | 0x4 << 6 | 20,

    PWM_PIN_A6_M1_G0 = SYSCTL_PERIPH_GPIOA << 19 | 1 << 18 | GPIO_PIN_6 << 10 | 0x8 << 6 | 12,
    PWM_PIN_A7_M1_G0 = SYSCTL_PERIPH_GPIOA << 19 | 1 << 18 | GPIO_PIN_7 << 10 | 0xc << 6 | 13,
    PWM_PIN_D0_M1_G1 = SYSCTL_PERIPH_GPIOD << 19 | 1 << 18 | GPIO_PIN_0 << 10 | 0x0 << 6 | 0,
    PWM_PIN_D1_M1_G1 = SYSCTL_PERIPH_GPIOD << 19 | 1 << 18 | GPIO_PIN_1 << 10 | 0x4 << 6 | 1,
    PWM_PIN_E4_M1_G2 = SYSCTL_PERIPH_GPIOE << 19 | 1 << 18 | GPIO_PIN_4 << 10 | 0x0 << 6 | 12,
    PWM_PIN_E5_M1_G2 = SYSCTL_PERIPH_GPIOE << 19 | 1 << 18 | GPIO_PIN_5 << 10 | 0x4 << 6 | 13,
    PWM_PIN_F0_M1_G3 = SYSCTL_PERIPH_GPIOF << 19 | 1 << 18 | GPIO_PIN_0 << 10 | 0x0 << 6 | 24,
    PWM_PIN_F1_M1_G3 = SYSCTL_PERIPH_GPIOF << 19 | 1 << 18 | GPIO_PIN_1 << 10 | 0x4 << 6 | 25,
    PWM_PIN_F2_M1_G4 = SYSCTL_PERIPH_GPIOF << 19 | 1 << 18 | GPIO_PIN_2 << 10 | 0x4 << 6 | 36,
    PWM_PIN_F3_M1_G4 = SYSCTL_PERIPH_GPIOF << 19 | 1 << 18 | GPIO_PIN_3 << 10 | 0x4 << 6 | 37,
};

enum UART_Channel {
    UART0A = 0, UART1B, UART2D, UART3C, UART4C, UART5E, UART6D, UART7E
};

static uint32_t UART_Pinconfig[] = {
        0x00000001,
        0x00010001,
        0x00031801,
        0x00021801,
        0x00021001,
        0x00021001,
        0x00041001,
        0x00031001,
        0x00040001,
};

typedef struct PID {
    double kp;
    double ki;
    double kd;
    int32_t CNT_pre;
    int32_t CNT_cur;
    struct ctr {
        double cur;
        double pre;
        double aim;
    } ctr;
    struct error {
        double cur;
        double pre;
        double sum;
        double bia;
    } error;

} pid;

void SystemClock_Config();

uint32_t Get_Tick();

void Systick_IntHandler();

double PID_Realize(pid *ctrl);

void PID_Init(pid *ctrl, double kp, double ki, double kd, double aim);

void SysDelay_ms(uint32_t ms);

void SysDelay_us(uint32_t us);

uint8_t GPIO_Init(uint32_t ui32Port, uint8_t ui8Pins, uint8_t type);

uint8_t GPIO_INTInit(uint32_t ui32port, uint8_t ui8pins, uint32_t ui32IntType, void (*pfnIntHandler)(void));

void GPIO_WriteState(uint32_t ui32Port, uint8_t ui8Pins, uint8_t state);

uint8_t GPIO_ReadState(uint32_t ui32Port, uint8_t ui8Pins);

uint32_t PWM_Init(uint32_t ui32ClkDiv, uint32_t ui32PWMPin, uint32_t *ui32PWMOut, uint32_t *ui32Gen);

void PWM_FreqSet(uint32_t ui32Base, uint32_t ui32Gen, uint8_t NumofDiv, uint32_t Freq);

void PWM_DutySet(uint32_t ui32Base, uint32_t ui32Gen, uint32_t ui32PWMOut, uint8_t duty);

void UART_RetargetInit(void);

int UART_printf(const char *fmt, ...);

uint32_t UART_Init(uint32_t uart_channel);

uint8_t UART_RxINTInit(uint32_t uart_channel, void (*pfnHandler)(void));

uint8_t Timer_INTInit(uint32_t ui32Interrupt, uint32_t ui32Freq, void (*pfnHandler)(void));

uint8_t Encoder1_INTInit(uint32_t ui32PortA, uint8_t ui8PinA, uint32_t ui32PortB, uint8_t ui8PinB);

uint8_t Encoder2_INTInit(uint32_t ui32PortA, uint8_t ui8PinA, uint32_t ui32PortB, uint8_t ui8PinB);

void Encoder_IRQHandler(void);

void GPIOF0_IRQHandler(void);

#ifdef MPU6050

#include "math.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "inv_mpu.h"

#define q30 1073741824.0f

#endif

#ifdef U8g2

#include "u8g2.h"

#define Delay(x)            SysDelay_ms(x)
#define OLED_DC_GPIO_Port   GPIO_PORTA_BASE
#define OLED_DC_Pin         GPIO_PIN_3
#define OLED_NSS_GPIO_Port  GPIO_PORTA_BASE
#define OLED_NSS_Pin        GPIO_PIN_6
#define OLED_RES_GPIO_Port  GPIO_PORTA_BASE
#define OLED_RES_Pin        GPIO_PIN_7

uint8_t u8x8_byte_4wire_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr); // 我们移植的函数的原型
uint8_t u8x8_gpio_and_delay(U8X8_UNUSED u8x8_t *u8x8, U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int, U8X8_UNUSED void *arg_ptr); // 我们移植的函数的原型

#endif


#endif //TM4TEMPLATE_TM4_MY_H
