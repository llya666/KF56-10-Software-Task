/**
  ******************************************************************************
  * @file    main.c  （精简单文件版，直接替换 Core/Src/main.c）
  * @brief   KF56-10 软件第一题 · HAL 精简版
  *
  *  设计只有三句话：
  *   1. 主循环是一个 1 ms 软件帧，同时做 PWM、累加 g_ms、跑任务；
  *   2. 按键全部 EXTI，中断只置标志，主循环处理；
  *   3. 呼吸相位由 g_ms 算，不累加，所以不会被按键打断。
  *
  *  使用方法：
  *   - 用本文件替换 CubeMX 工程的 Core/Src/main.c；
  *   - 把 app.c 从工程移除（如果有）；
  *   - 编译、下载、按复位。
  ******************************************************************************
  */

#include "stm32f4xx_hal.h"

void Error_Handler(void);

/*============================ 1. 引脚 ============================*/
#define LED0_ON()   do { GPIOF->BSRR = ((uint32_t)GPIO_PIN_9 << 16); } while (0)
#define LED0_OFF()  do { GPIOF->BSRR =  (uint32_t)GPIO_PIN_9;        } while (0)
#define LED1_ON()   do { GPIOF->BSRR = ((uint32_t)GPIO_PIN_10 << 16); } while (0)
#define LED1_OFF()  do { GPIOF->BSRR =  (uint32_t)GPIO_PIN_10;        } while (0)
#define BEEP_ON()   do { GPIOF->BSRR =  (uint32_t)GPIO_PIN_8;         } while (0)
#define BEEP_OFF()  do { GPIOF->BSRR = ((uint32_t)GPIO_PIN_8 << 16);  } while (0)
#define WAVE_ON()   do { GPIOA->BSRR =  (uint32_t)GPIO_PIN_3;         } while (0)
#define WAVE_OFF()  do { GPIOA->BSRR = ((uint32_t)GPIO_PIN_3 << 16);  } while (0)

/*============================ 2. 参数 ============================*/
#define SLOT_US        10u      /* 1 个时隙 10 µs */
#define LOOPS_PER_US   24u      /* 校准值：示波器 PA3 调到 1 kHz */
#define DEBOUNCE_MS    50u       /* 消抖：两次有效边沿的最小间隔 */
#define CLICK_MS       350u     /* 多击窗口 */
#define KEY1_MS        2000u
#define CD_MS          3000u
#define CD_HALF_MS     500u     /* 1 Hz */
#define BLINK_HALF_MS  250u     /* 500 ms 完整周期 */
#define BREATH_STEP_MS 10u

/*============================ 3. 状态 ============================*/
static volatile uint32_t g_ms;
static volatile uint8_t  ev0, ev1, ev2, ev_reset;
static uint32_t last0, last1, last2;
static uint8_t  seen0, seen1, seen2;

static uint8_t  g_mode = 1;     /* 1~6 */
static uint32_t t_mode, t_breath;
static uint32_t t_key1, t_cd, t_click, t_chirp, t_flash;
static uint8_t  g_key1, g_cd, g_beep;   /* bit0=KEY1, bit1=长鸣, bit2=提示音 */
static uint8_t  g_click, g_cnt;
static uint8_t  g_chirp_idx, g_chirp_on;
static const uint16_t pat_a[] = { 80, 80, 80, 80, 200 };            /* 滴滴—— */
static const uint16_t pat_b[] = { 60, 60, 60, 60, 60, 60, 300 };    /* 滴滴滴—— */
/* 6 击“演奏”：小星星节奏，慢速版，约 11 秒
 * 每段 6 短 + 1 长 + 150 ms 停顿 = 2.75 s，重复 4 段；
 * 每段 14 个时长（偶数），保证下一段从“响”开始 */
static const uint16_t pat_c[] = {
    200, 150, 200, 150, 200, 150, 200, 150, 200, 150, 200, 150, 500, 150,
    200, 150, 200, 150, 200, 150, 200, 150, 200, 150, 200, 150, 500, 150,
    200, 150, 200, 150, 200, 150, 200, 150, 200, 150, 200, 150, 500, 150,
    200, 150, 200, 150, 200, 150, 200, 150, 200, 150, 200, 150, 500, 150
};
static const uint16_t *g_pat = pat_a;
static uint8_t  g_pat_len = 5;
static uint8_t  g_flash, g_flash_on;    /* 第 4 击：LED1 快闪 3 次 */
static uint8_t  g_wave = 50;    /* 模式 6 占空比 0~100 */
static uint8_t  d0, d1, dw;     /* 三路 PWM 占空比 */

/*============================ 4. 软件延时 ========================*/
static void delay_us(uint32_t us)
{
    volatile uint32_t n = us * LOOPS_PER_US;
    while (n) { n--; __NOP(); }
}

/*============================ 5. GPIO / EXTI =====================*/
static void gpio_init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
    HAL_GPIO_Init(GPIOF, &g);
    g.Pin = GPIO_PIN_3;
    HAL_GPIO_Init(GPIOA, &g);
    LED0_OFF(); LED1_OFF(); BEEP_OFF(); WAVE_OFF();

    g.Mode = GPIO_MODE_IT_FALLING;
    g.Pull = GPIO_PULLUP;
    g.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4;
    HAL_GPIO_Init(GPIOE, &g);

    g.Mode = GPIO_MODE_IT_RISING;
    g.Pull = GPIO_PULLDOWN;
    g.Pin = GPIO_PIN_0;
    HAL_GPIO_Init(GPIOA, &g);

    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4);

    HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0); HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    HAL_NVIC_SetPriority(EXTI4_IRQn, 1, 0); HAL_NVIC_EnableIRQ(EXTI4_IRQn);
    HAL_NVIC_SetPriority(EXTI3_IRQn, 2, 0); HAL_NVIC_EnableIRQ(EXTI3_IRQn);
    HAL_NVIC_SetPriority(EXTI2_IRQn, 3, 0); HAL_NVIC_EnableIRQ(EXTI2_IRQn);
}

/* 50 ms 时间戳消抖：按住只有一个下降沿，所以天然不连跳 */
static void key_accept(volatile uint8_t *ev, uint32_t *last, uint8_t *seen)
{
    uint32_t now = g_ms;
    if (!*seen || (uint32_t)(now - *last) >= DEBOUNCE_MS) {
        *seen = 1; *last = now; *ev = 1;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    if (pin == GPIO_PIN_0) {                 /* WK_UP：先硬关，再置复位 */
        LED0_OFF(); LED1_OFF(); BEEP_OFF(); WAVE_OFF();
        ev_reset = 1;
    } else if (pin == GPIO_PIN_2) {          /* KEY2 */
        key_accept(&ev2, &last2, &seen2);
    } else if (pin == GPIO_PIN_3) {          /* KEY1 */
        key_accept(&ev1, &last1, &seen1);
    } else if (pin == GPIO_PIN_4) {          /* KEY0 */
        key_accept(&ev0, &last0, &seen0);
    }
}

static uint8_t take(volatile uint8_t *e)
{
    uint8_t v;
    uint32_t p = __get_PRIMASK();
    __disable_irq();
    v = *e; *e = 0;
    __set_PRIMASK(p);
    return v;
}

/*============================ 6. 业务逻辑 =========================*/
static void mode_set(uint8_t m)
{
    g_mode = m;
    t_mode = g_ms;
    t_breath = g_ms;
    g_key1 = 0; g_cd = 0; g_beep = 0;
    g_click = 0; g_cnt = 0;
    g_flash = 0; g_flash_on = 0;
    d0 = d1 = dw = 0;
    BEEP_OFF(); WAVE_OFF();
}

static void reset_all(void)
{
    mode_set(1);
    d0 = d1 = dw = 0;
    LED0_OFF(); LED1_OFF(); BEEP_OFF(); WAVE_OFF();
}

/* 基本功能 2：鸣叫中再按忽略；新动作覆盖旧长鸣 */
static void key1_start(void)
{
    if (g_key1) return;
    g_key1 = 1;
    t_key1 = g_ms;
    g_beep = (uint8_t)((g_beep & ~6u) | 1u);   /* 清长鸣/提示音，置 2 s 报警 */
}

/* 基本功能 3：倒计时期间再按忽略 */
static void cd_start(void)
{
    if (g_cd) return;
    g_cd = 1;
    t_cd = g_ms;
    g_beep &= (uint8_t)~6u;   /* 清长鸣/提示音 */
}

static void click_add(void)
{
    if (g_cnt >= 6) return;   /* 最多 6 击，7 击以上忽略 */
    g_cnt++;
    t_click = g_ms;
    g_click = 1;
}

/* 第 4 击：LED1 快闪 3 次（LED0 继续呼吸） */
static void led1_flash(void)
{
    g_flash = 6;              /* 6 次翻转 = 3 个亮灭周期 */
    g_flash_on = 1;
    t_flash = g_ms;
}

/* 启动一段节奏：p 是时长数组（开、关、开、关……），len 是段数 */
static void chirp_start(const uint16_t *p, uint8_t len)
{
    g_pat = p;
    g_pat_len = len;
    g_chirp_idx = 0;
    g_chirp_on = 1;
    t_chirp = g_ms;
    g_beep |= 4u;
}

/* 模式 6：一次事件只走 5%，按住不连跳 */
static void wave_step(int8_t d)
{
    if (d > 0) g_wave = (g_wave >= 95) ? 100 : (uint8_t)(g_wave + 5);
    else       g_wave = (g_wave <= 5)  ? 0   : (uint8_t)(g_wave - 5);
}

static void display(void)
{
    uint8_t a = 0, b = 0, w = 0;
    uint32_t el = g_ms - t_mode;

    uint16_t ph = (uint16_t)(((g_ms - t_breath) / BREATH_STEP_MS) % 200u);
    uint8_t  lv = (ph <= 100u) ? (uint8_t)ph : (uint8_t)(200u - ph);
    uint8_t  br = (uint8_t)((uint16_t)lv * lv / 100u);   /* 二次曲线，渐变平滑 */

    switch (g_mode) {
    case 1: { uint8_t h = (uint8_t)((el / BLINK_HALF_MS) & 1u); a = h ? 0 : 100; b = h ? 100 : 0; break; }
    case 2: { uint8_t h = (uint8_t)((el / BLINK_HALF_MS) & 1u); a = b = h ? 0 : 100; break; }
    case 3: a = br; break;
    case 4: b = br; break;
    case 5: a = br; break;
    case 6: a = g_wave; w = g_wave; break;
    default: break;
    }

    /* 模式 5 不覆盖呼吸，保证“呼吸不得停顿” */
    if (g_key1 && g_mode != 5) a = 100;

    if (g_cd && g_mode != 6) {
        uint32_t e = g_ms - t_cd;
        b = ((e / CD_HALF_MS) & 1u) ? 0 : 100;
    }

    if (g_flash) b = g_flash_on ? 100 : 0;   /* 第 4 击：LED1 快闪，LED0 不受影响 */

    d0 = a; d1 = b; dw = w;

    if (g_beep & 3u) {                  /* KEY1 报警 / 长鸣优先，持续响 */
        BEEP_ON();
    } else if (g_beep & 4u) {           /* 第 4 击提示音：按节奏开关 */
        if (g_chirp_on) BEEP_ON(); else BEEP_OFF();
    } else {
        BEEP_OFF();
    }
}

static void task_1ms(void)
{
    if (take(&ev_reset)) {
        ev0 = ev1 = ev2 = 0;
        reset_all();
        return;
    }

    if (take(&ev0)) mode_set((uint8_t)((g_mode % 6) + 1));

    if (take(&ev1)) {
        if (g_mode == 6) wave_step(1); else key1_start();
    }

    if (take(&ev2)) {
        if (g_mode == 6) wave_step(-1);
        else if (g_mode == 5) click_add();
        else cd_start();
    }

    if (g_key1 && (uint32_t)(g_ms - t_key1) >= KEY1_MS) { g_key1 = 0; g_beep &= (uint8_t)~1u; }
    if (g_cd   && (uint32_t)(g_ms - t_cd)   >= CD_MS)   { g_cd = 0;   g_beep |= 2u; }
    if (g_beep & 4u) {
        if ((uint32_t)(g_ms - t_chirp) >= g_pat[g_chirp_idx]) {
            t_chirp = g_ms;
            g_chirp_idx++;
            if (g_chirp_idx >= g_pat_len) {
                g_beep &= (uint8_t)~4u;      /* 节奏播完 */
            } else {
                g_chirp_on = (uint8_t)!g_chirp_on;
            }
        }
    }

    if (g_flash) {
        if ((uint32_t)(g_ms - t_flash) >= 100u) {
            t_flash = g_ms;
            g_flash_on = (uint8_t)!g_flash_on;
            g_flash--;
            if (g_flash == 0) g_flash_on = 0;   /* 闪完保持灭 */
        }
    }

    if (g_click && (uint32_t)(g_ms - t_click) >= CLICK_MS) {
        uint8_t n = g_cnt;
        g_click = 0; g_cnt = 0;
        if (n == 1) key1_start();
        else if (n == 2) cd_start();
        else if (n == 3) reset_all();
        else if (n == 4) {
            led1_flash();
            if ((g_beep & 3u) == 0) chirp_start(pat_a, 5);
        }
        else if (n == 5) {
            if ((g_beep & 3u) == 0) chirp_start(pat_b, 7);
        }
        else if (n == 6) {
            if ((g_beep & 3u) == 0) {
                chirp_start(pat_c, (uint8_t)(sizeof(pat_c) / sizeof(pat_c[0])));
            }
        }
    }

    display();
}

/*============================ 7. 1 ms 软件帧 =====================*/
static void frame_1ms(void)
{
    uint8_t i;
    for (i = 0; i < 100u; i++) {
        if (ev_reset) {
            LED0_OFF(); LED1_OFF(); BEEP_OFF(); WAVE_OFF();
        } else {
            if (i < d0) LED0_ON(); else LED0_OFF();
            if (i < d1) LED1_ON(); else LED1_OFF();
            if (i < dw) WAVE_ON(); else WAVE_OFF();
        }
        delay_us(SLOT_US);
    }
    g_ms++;
    task_1ms();
}

/*============================ 8. 时钟 / 初始化 ===================*/
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef o = {0};
    RCC_ClkInitTypeDef c = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    o.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    o.HSIState = RCC_HSI_ON;
    o.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    o.PLL.PLLState = RCC_PLL_ON;
    o.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    o.PLL.PLLM = 16; o.PLL.PLLN = 336;
    o.PLL.PLLP = RCC_PLLP_DIV2; o.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&o) != HAL_OK) Error_Handler();

    c.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    c.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    c.AHBCLKDivider = RCC_SYSCLK_DIV1;
    c.APB1CLKDivider = RCC_HCLK_DIV4;
    c.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&c, FLASH_LATENCY_5) != HAL_OK) Error_Handler();
}

static void app_init(void)
{
    __disable_irq();
    g_ms = 0; g_mode = 1; t_mode = 0; t_breath = 0;
    g_key1 = 0; g_cd = 0; g_beep = 0;
    g_click = 0; g_cnt = 0; t_chirp = 0;
    g_chirp_idx = 0; g_chirp_on = 0;
    g_flash = 0; g_flash_on = 0; t_flash = 0;
    g_pat = pat_a; g_pat_len = 5;
    g_wave = 50; d0 = d1 = dw = 0;
    gpio_init();
    __enable_irq();
}

/*============================ 9. 中断入口 / main =================*/
void EXTI0_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0); }
void EXTI2_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2); }
void EXTI3_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3); }
void EXTI4_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4); }

HAL_StatusTypeDef HAL_InitTick(uint32_t p) { (void)p; return HAL_OK; }  /* 不启动 SysTick */

void Error_Handler(void) { __disable_irq(); while (1) { } }

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    app_init();
    while (1) {
        frame_1ms();
    }
}
