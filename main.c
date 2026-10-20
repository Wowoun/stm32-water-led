/*==================================================================================
 *  课后作业 · 流水灯   ——  STM32F103C8T6 最小系统板
 *  ------------------------------------------------------------------------------
 *  学号 / 姓名：____________________
 *
 *  实现内容：
 *    1) 上电自检  : 4 个 LED 依次点亮一圈，确认接线正确
 *    2) 流水灯    : LED 从 PB12 依次流向 PB15，首尾相接循环
 *    3) 进阶功能  : 短按按键（PB0）改变流水方向；长按按键切换流水速度
 *
 *  硬件连线约定（面包板）：
 *    LED 阳极(长脚) --[1K 电阻]-- 3.3V ，LED 阴极(短脚) -- GPIO
 *      => GPIO 输出低电平点亮（与板载 PC13 灯同逻辑）
 *    按键一端 -- GPIO ，另一端 -- GND ，GPIO 内部上拉
 *      => 空闲读 1，按下读 0
 *
 *  技术要点：
 *    - 纯寄存器操作，不依赖 HAL 库（与 01-点灯 工程保持一致）
 *    - 时基用 SysTick 1ms 中断（非忙等），所以按键响应不受流水延时影响
 *    - GPIO 写 BSRR / BRR，原子置位/复位，避免"读-改-写"的竞争
 *    - 按键软件消抖 20ms，长按判定 1.5s
 *
 *  引脚选择理由（避免与板载外设冲突）：
 *    PB12~PB15 —— 避开板载 LED(PA1/PA2)、板载按键(PB1/PB11)、
 *                 SWD 调试口(PA13/PA14)、串口(PA9/PA10)、SPI(PA4~PA7)
 *    PB0       —— 同上理由；PB3/PB4 是 JTAG 口也避开了
 *
 *  编译环境：Keil µVision 5.43.1 + AC6 (armclang 6.24)
 *              Device=STM32F103C8  Define=STM32F10X_MD  (SystemCoreClock = 72MHz)
 *================================================================================*/

#include "stm32f10x.h"

/* 本工程只带 CMSIS 头文件（无标准外设库），故自行定义引脚位掩码 */
#define PIN_MASK(n)          ((uint32_t)1u << (n))

/*----------------------------------------------------------------------------------
 *  一、引脚分配表
 *--------------------------------------------------------------------------------*/
#define LED_COUNT            4u

typedef struct {
    GPIO_TypeDef *port;   /* LED 所在的 GPIO 端口 */
    uint32_t      pin;    /* 引脚掩码，取 GPIO_Pin_x */
} led_t;

static const led_t s_led[LED_COUNT] = {
    { GPIOB, PIN_MASK(12) },   /* LED1 -> PB12 */
    { GPIOB, PIN_MASK(13) },   /* LED2 -> PB13 */
    { GPIOB, PIN_MASK(14) },   /* LED3 -> PB14 */
    { GPIOB, PIN_MASK(15) },   /* LED4 -> PB15 */
};

#define KEY_PORT             GPIOB
#define KEY_PIN              PIN_MASK(0)    /* 按键 K1 -> PB0 */

/*----------------------------------------------------------------------------------
 *  二、时基：SysTick 1ms 中断
 *--------------------------------------------------------------------------------*/
static volatile uint32_t s_tick = 0u;

void SysTick_Handler(void)
{
    s_tick++;
}

static uint32_t millis(void)
{
    return s_tick;
}

static void delay_ms(uint32_t ms)
{
    uint32_t t0 = millis();
    while ((millis() - t0) < ms) {
        /* 等待，不阻塞中断 */
    }
}

static void systick_init(void)
{
    /* 用实际主频算，避免晶振没起振时（退化成 HSI 8MHz）延时偏差 9 倍 */
    SysTick->LOAD = (SystemCoreClock / 1000u) - 1u;
    SysTick->VAL  = 0u;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

/*----------------------------------------------------------------------------------
 *  三、GPIO 底层配置
 *--------------------------------------------------------------------------------*/

/* 通用推挽输出。cfg = 0x2 表示 MODE=10(2MHz) CNF=00(推挽) */
static void gpio_out_init(GPIO_TypeDef *port, uint32_t pinmask, uint32_t cfg)
{
    uint32_t crl = port->CRL;
    uint32_t crh = port->CRH;
    uint32_t p;

    for (p = 0u; p < 16u; p++) {
        uint32_t bit = 1u << p;
        uint32_t *reg;
        uint32_t shift;

        if ((pinmask & bit) == 0u) {
            continue;
        }
        reg   = (p < 8u) ? &crl : &crh;
        shift = (p < 8u) ? (p * 4u) : ((p - 8u) * 4u);
        *reg  = (*reg & ~(0xFu << shift)) | (cfg << shift);
    }
    port->CRL = crl;
    port->CRH = crh;
}

/* 内部上拉输入：MODE=00 CNF=10，ODR=1 选中上拉 */
static void gpio_in_pullup_init(GPIO_TypeDef *port, uint32_t pinmask)
{
    uint32_t crl = port->CRL;
    uint32_t crh = port->CRH;
    uint32_t p;

    for (p = 0u; p < 16u; p++) {
        uint32_t bit = 1u << p;
        uint32_t *reg;
        uint32_t shift;

        if ((pinmask & bit) == 0u) {
            continue;
        }
        reg   = (p < 8u) ? &crl : &crh;
        shift = (p < 8u) ? (p * 4u) : ((p - 8u) * 4u);
        *reg  = (*reg & ~(0xFu << shift)) | (0x8u << shift);   /* 0x8 = MODE00 CNF10 */
    }
    port->CRL = crl;
    port->CRH = crh;
    port->BSRR = pinmask;   /* ODR 置 1 -> 选中内部上拉 */
}

/*----------------------------------------------------------------------------------
 *  四、LED 与按键
 *--------------------------------------------------------------------------------*/
static void led_init(void)
{
    uint8_t i;

    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

    for (i = 0u; i < LED_COUNT; i++) {
        gpio_out_init(s_led[i].port, s_led[i].pin, 0x2u);
    }
    for (i = 0u; i < LED_COUNT; i++) {
        s_led[i].port->BSRR = s_led[i].pin;   /* 全部熄灭 */
    }
}

static void led_on(uint8_t idx)
{
    s_led[idx].port->BRR  = s_led[idx].pin;  /* 拉低 -> 点亮 */
}

static void led_off(uint8_t idx)
{
    s_led[idx].port->BSRR = s_led[idx].pin;  /* 拉高 -> 熄灭 */
}

/* 按键事件 */
#define KEY_EV_PRESS   0x01u   /* 短按（松开时产生） */
#define KEY_EV_LONG    0x02u   /* 长按 1.5s（按住时产生） */

#define KEY_DEBOUNCE_MS    20u
#define KEY_LONG_MS     1500u

typedef struct {
    GPIO_TypeDef *port;
    uint32_t      pin;
    uint8_t       last_raw;    /* 上一次原始采样：1 松开 / 0 按下 */
    uint8_t       stable;      /* 消抖后的稳定状态 */
    uint32_t      t_change;    /* 最近一次原始电平跳变时刻 */
    uint32_t      t_press;     /* 确认按下的时刻 */
    uint8_t       long_sent;   /* 本次按下是否已发过长按事件 */
} key_t;

static key_t s_key;

static void key_init(void)
{
    s_key.port      = KEY_PORT;
    s_key.pin       = KEY_PIN;
    s_key.last_raw  = 1u;
    s_key.stable    = 1u;
    s_key.t_change  = 0u;
    s_key.t_press   = 0u;
    s_key.long_sent = 0u;

    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    gpio_in_pullup_init(KEY_PORT, KEY_PIN);
}

static uint8_t key_poll(key_t *k)
{
    uint8_t  raw = ((k->port->IDR & k->pin) != 0u) ? 1u : 0u;
    uint8_t  ev  = 0u;
    uint32_t now = millis();

    if (raw != k->last_raw) {
        /* 刚发生电平跳变：重新计时，等待稳定 */
        k->last_raw = raw;
        k->t_change = now;
    } else if ((raw != k->stable) && ((now - k->t_change) >= KEY_DEBOUNCE_MS)) {
        /* 持续超过消抖时间，认定为稳定状态变化 */
        k->stable = raw;
        if (k->stable == 0u) {
            k->t_press   = now;      /* 确认按下 */
            k->long_sent = 0u;
        } else {
            if (k->long_sent == 0u) {
                ev |= KEY_EV_PRESS;  /* 短按：松开时结算 */
            }
            k->long_sent = 0u;
        }
    }

    if ((k->stable == 0u) && (k->long_sent == 0u) && ((now - k->t_press) >= KEY_LONG_MS)) {
        k->long_sent = 1u;
        ev |= KEY_EV_LONG;           /* 长按：按住 1.5s 立即结算 */
    }

    return ev;
}

/*----------------------------------------------------------------------------------
 *  五、主程序
 *--------------------------------------------------------------------------------*/
static const uint32_t s_speed[3] = { 300u, 150u, 60u };   /* 慢 / 中 / 快，单位 ms */

int main(void)
{
    uint8_t idx      = 0u;
    int8_t  dir      = 1;      /* +1 正向(PB12->PB15)  -1 反向 */
    uint8_t speed_ix = 1u;
    uint8_t i;
    uint32_t t;

    led_init();
    key_init();
    systick_init();

    /* 上电自检：4 个 LED 依次点亮一圈 */
    for (i = 0u; i < LED_COUNT; i++) {
        led_on(i);
        delay_ms(150u);
        led_off(i);
    }
    delay_ms(300u);

    while (1) {
        led_on(idx);

        /* 在一个流水节拍内持续轮询按键，保证按键响应不受延时影响 */
        t = millis();
        while ((millis() - t) < s_speed[speed_ix]) {
            uint8_t ev = key_poll(&s_key);

            if ((ev & KEY_EV_PRESS) != 0u) {
                dir = (int8_t)-dir;                       /* 短按：改变流水方向 */
            }
            if ((ev & KEY_EV_LONG) != 0u) {
                speed_ix = (uint8_t)((speed_ix + 1u) % 3u); /* 长按：切换速度 */
            }
        }

        led_off(idx);
        idx = (uint8_t)((idx + (int8_t)dir + (int8_t)LED_COUNT) % (int8_t)LED_COUNT);
    }
}
