// guus 2023. very simple ltc/smpte timecode receiver/decoder for ch32x033. ltc attached to pin A6. uses the first exti itc(7-0).
// ltc is pretty well explained here: https://en.wikipedia.org/wiki/Linear_timecode
// !cannot be used icm with debug.h's delay function, since it resets the systick counter!
// call ltc_begin set gpio and interrupt.
// if ltc_receiving() returns 0xff,
// ltc_lastFrame()->*stuff will be last received frame, see LTC_DATA struct for contents

#ifndef ltc_receiver_h
#define ltc_receiver_h
#include <ch32x035_exti.h>
#include <ch32x035_gpio.h>
#include <ch32x035_rcc.h>
#define LTC_FWDPATTERN 0b0011111111111101

#define LTC_REVPATTERN 0b1011111111111100
#define GPIOPIN GPIO_Pin_6
#define GPIOPORT GPIOA

#include <ssd1306_ch32v.h>
typedef enum LTC_DIR
{
    LTC_DIR_REVERSE = 0x00,
    LTC_DIR_FORWARD = 0x01,
} LTC_DIR;

typedef enum LTC_DROPFRAME
{
    LTC_DF_NON = 0x00,
    LTC_DF_DROP = 0x01,
} LTC_DROPFRAME;

typedef enum LTC_FRAMRATE
{
    LTC_FPS24 = 24,
    LTC_FPS25 = 25,
    LTC_FPS30 = 30,

} LTC_FRAMRATE;

typedef enum LTC_STATE
{
    LTC_ST_UNSYNC,
    LTC_ST_TIMECODE,
    LTC_ST_SYNCOWRD,

} LTC_STATE;

typedef enum LTC_DECODEPART
{
    LTC_DECODE_DIR = 0,
    LTC_DECODE_FRAME = 1,
    LTC_DECODE_SEC = 2,
    LTC_DECODE_MIN = 3,
    LTC_DECODE_HR = 4,
    LTC_DECODE_FRATE = 5,

} LTC_DECODEPART;

typedef struct LTC_DATA
{
    unsigned char hours;
    unsigned char minutes;
    unsigned char seconds;
    unsigned char frames;
    unsigned char user1;
    unsigned char user2;
    unsigned char user3;
    unsigned char user4;
    LTC_FRAMRATE framerate;
    LTC_DROPFRAME dropframe;
    LTC_DIR direction;
} LTC_DATA;

volatile LTC_DATA ltc_dataA;
volatile LTC_DATA ltc_dataB;

volatile LTC_DATA *ltc_data = &ltc_dataA;
volatile LTC_DATA *ltc_FrameComplete = &ltc_dataB;

volatile LTC_DIR ltc_direction = LTC_DIR_FORWARD;
unsigned short ltc_timecodeData = 0;
unsigned short ltc_timecodeDataCpy = 0;
volatile unsigned long long ltc_lastStk = 0;
volatile unsigned long long ltc_beginFrame = 0;
volatile unsigned long ltc_delta = 0;
volatile unsigned short ltc_syncword = 0;
volatile unsigned short ltc_syncwordCpy = 0;
volatile LTC_STATE ltc_state;
volatile unsigned char ltc_bitcount;
volatile unsigned char ltc_secondOneTransition = 0x00;
volatile unsigned short itc_count = 0;
volatile unsigned char ltc_newFrame = 0;

unsigned long ltc_oneMintime;
unsigned long ltc_zeroMintime;
unsigned long ltc_zeroMaxtime;
unsigned long ltc_maxFPS24;
unsigned long ltc_maxFPS25;

void ltc_begin()
{

    GPIO_InitTypeDef gpioin;
    EXTI_InitTypeDef extitc;

    unsigned long ticksps;

    // if systick is not already running
    if (!(SysTick->SR & 0x00000001))
    {
        // disable swi
        SysTick->CTLR &= ~(0x80000000);
        // autoreload to count up
        SysTick->CTLR &= ~(0x00000010);
        // autoreload to continue
        SysTick->CTLR &= ~(0x00000008);
        // hclk /8
        SysTick->CTLR &= ~(0x00000004);
        // disable interupt
        SysTick->CTLR &= ~(0x00000002);
        // start systick
        SysTick->CTLR |= 0x00000001;
    }
    else
    {
        // // disable swi
        // SysTick->CTLR &= ~(0x80000000);
        // autoreload to count up
        SysTick->CTLR &= ~(0x00000010);
        // autoreload to continue
        SysTick->CTLR &= ~(0x00000008);
    }

    ticksps = SystemCoreClock;
    if (!(SysTick->CTLR & 0x00000004))
        ticksps = ticksps / 8;

    // at 24fps a zero is  960hz, so approx 1050microsec
    // at 30fps a one is 2400hz, so approx 416micros
    // to be a litte more safe lets just say:
    // max time zero is 1200usec
    // min time zero is 350usec
    // these vals seem to work fine with all framerates
    ticksps = ticksps / 1000000;

    ltc_oneMintime = 50 * ticksps;
    ltc_zeroMintime = 380 * ticksps;
    ltc_zeroMaxtime = 1500 * ticksps;

    ltc_maxFPS24 = 80 * 510 * ticksps;
    ltc_maxFPS25 = 80 * 450 * ticksps;

    gpioin.GPIO_Pin = GPIOPIN;
    gpioin.GPIO_Mode = GPIO_Mode_IPD;
    gpioin.GPIO_Speed = GPIO_Speed_50MHz;

    extitc.EXTI_Line = EXTI_Line6;
    extitc.EXTI_Mode = EXTI_Mode_Interrupt;
    extitc.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    extitc.EXTI_LineCmd = ENABLE;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

    EXTI_Init(&extitc);
    GPIO_Init(GPIOPORT, &gpioin);
    NVIC_EnableIRQ(EXTI7_0_IRQn);
    NVIC_SetPriority(EXTI7_0_IRQn, 0x80);
}

unsigned char ltc_receiving()
{
    unsigned char r = 0;
    if (((SysTick->CNT - ltc_lastStk) > ltc_zeroMaxtime) || (ltc_state == LTC_ST_UNSYNC))
    {
        r = 0;
        ltc_newFrame = 0x00;
        ltc_state = LTC_ST_UNSYNC;
    }
    else
        r = 0xff;
    return r;
}

LTC_DATA *ltc_lastFrame()
{
    return ltc_FrameComplete;
}

unsigned char ltc_reverseNible(unsigned char in)
{
    unsigned char r = 0;
    if (in & 0x01)
        r |= 0x08;
    if (in & 0x02)
        r |= 0x04;
    if (in & 0x04)
        r |= 0x02;
    if (in & 0x08)
        r |= 0x01;
    return r;
}

// does the binairy-decimal decoding stuf... chopped in 6 steps, see decodepart
unsigned char ltc_decode(unsigned long data, LTC_DATA *p, LTC_DECODEPART s)
{

    unsigned char r = 0;
    switch (s)
    {
    case LTC_DECODE_DIR:

        if (data == LTC_FWDPATTERN)
            p->direction = LTC_DIR_FORWARD;
        else if (data == LTC_REVPATTERN)
            p->direction = LTC_DIR_REVERSE;
        else
            r = 1;
        break;

    case LTC_DECODE_FRAME:
        p->frames = ltc_reverseNible((data >> 12) & 0x0f);
        p->frames += (ltc_reverseNible((data >> 4) & 0x0c) & 0x03) * 10;

        p->user1 = ltc_reverseNible((data >> 8) & 0x0f);
        p->user1 |= (ltc_reverseNible((data) & 0x0f) << 4);

        if ((data >> 5) & 0x0001)
            p->dropframe = LTC_DF_DROP;
        else
            p->dropframe = LTC_DF_NON;

        if (p->frames > 30)
            r = 2;
        break;

    case LTC_DECODE_SEC:
        p->seconds = ltc_reverseNible((data >> 12) & 0x0f);
        p->seconds += (ltc_reverseNible((data >> 4) & 0x0e) & 0x07) * 10;

        p->user2 = ltc_reverseNible((data >> 8) & 0x0f);
        p->user2 |= (ltc_reverseNible((data) & 0x0f) << 4);

        if (p->seconds > 59)
            r = 2;
        break;

    case LTC_DECODE_MIN:
        p->minutes = ltc_reverseNible((data >> 12) & 0x0f);
        p->minutes += (ltc_reverseNible((data >> 4) & 0x0e) & 0x07) * 10;

        p->user3 = ltc_reverseNible((data >> 8) & 0x0f);
        p->user3 |= (ltc_reverseNible((data) & 0x0f) << 4);

        if (p->minutes > 59)
            r = 3;
        break;

    case LTC_DECODE_HR:
        p->hours = ltc_reverseNible((data >> 12) & 0x0f);
        p->hours += (ltc_reverseNible((data >> 4) & 0x0c) & 0x03) * 10;

        p->user4 = ltc_reverseNible((data >> 8) & 0x0f);
        p->user4 |= (ltc_reverseNible((data) & 0x0f) << 4);

        if (p->hours > 23)
            r = 2;
        break;

    case LTC_DECODE_FRATE:
        if (data > ltc_maxFPS24)
            p->framerate = LTC_FPS24;
        else if (data > ltc_maxFPS25)
            p->framerate = LTC_FPS25;
        else
            p->framerate = LTC_FPS30;
        break;
    default:
        break;
    }
    return 0;
}

void ltc_itc(void)
{
    volatile unsigned char newbit = 0x0000;
    volatile unsigned long long time = SysTick->CNT;
    ltc_delta = time - ltc_lastStk;

    // bad 'debounce'
    if (ltc_delta < ltc_oneMintime)
        return;

    // save for next time
    ltc_lastStk = time;

    if (ltc_delta > ltc_zeroMaxtime)
    {
        ltc_state = LTC_ST_UNSYNC;
        return;
    }
    if (ltc_delta < ltc_zeroMintime)
        newbit = 0x0001;
    if (ltc_secondOneTransition) // one hs been set previous
    {
        ltc_secondOneTransition = 0x00;
        return;
    }
    else
    {
        itc_count++;

        if (newbit)
            ltc_secondOneTransition = 0xff;

        switch (ltc_state)
        {
        case LTC_ST_UNSYNC:
            ltc_syncword = ltc_syncword << 1;
            ltc_syncword |= newbit;

            if ((ltc_syncword == LTC_FWDPATTERN) || ((ltc_syncword == LTC_REVPATTERN)))
            {
                ltc_bitcount = 0;
                ltc_syncword = 0x0000;
                ltc_state = LTC_ST_TIMECODE;
                ltc_timecodeData = 0;
            }
            break;

        case LTC_ST_TIMECODE:

            ltc_timecodeData = ltc_timecodeData << 1;
            ltc_timecodeData |= newbit;

            ltc_bitcount++;
            if (ltc_bitcount == 1)
            {
                ltc_beginFrame = ltc_lastStk;
            }
            if (ltc_bitcount == 16)
            {
                ltc_timecodeDataCpy = ltc_timecodeData;
                ltc_decode(ltc_timecodeDataCpy, ltc_data, LTC_DECODE_FRAME);
                ltc_timecodeData = 0;
            }
            if (ltc_bitcount == 32)
            {
                ltc_timecodeDataCpy = ltc_timecodeData;
                ltc_decode(ltc_timecodeData, ltc_data, LTC_DECODE_SEC);
                ltc_timecodeData = 0;
            }
            if (ltc_bitcount == 48)
            {
                ltc_timecodeDataCpy = ltc_timecodeData;
                ltc_decode(ltc_timecodeData, ltc_data, LTC_DECODE_MIN);
                ltc_timecodeData = 0;
            }
            if (ltc_bitcount == 64)
            {
                ltc_timecodeDataCpy = ltc_timecodeData;
                ltc_decode(ltc_timecodeData, ltc_data, LTC_DECODE_HR);
                ltc_bitcount = 0;
                ltc_state = LTC_ST_SYNCOWRD;
                ltc_timecodeData = 0;
            }
            break;

        case LTC_ST_SYNCOWRD:
            ltc_syncword = ltc_syncword << 1;
            ltc_syncword |= newbit;

            ltc_bitcount++;
            if (ltc_bitcount == 16)
            {
                ltc_bitcount = 0;
                if ((ltc_syncword == LTC_FWDPATTERN) || ((ltc_syncword == LTC_REVPATTERN)))
                {
                    ltc_syncwordCpy = ltc_syncword;
                    ltc_syncword = 0x0000;
                    ltc_state = LTC_ST_TIMECODE;
                    ltc_decode(ltc_syncwordCpy, ltc_data, LTC_DECODE_DIR);
                    ltc_decode((time - ltc_beginFrame), ltc_data, LTC_DECODE_FRATE);
                    // swapt he pointer
                    LTC_DATA *tmp;
                    tmp = ltc_data;
                    ltc_data = ltc_FrameComplete;
                    ltc_FrameComplete = tmp;
                    ltc_newFrame = 0xff;
                }
                else
                {
                    ltc_syncword = 0x0000;
                    ltc_state = LTC_ST_UNSYNC;
                }
            }
            break;
        default:
            break;
        }
    }
}

extern void EXTI7_0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI7_0_IRQHandler(void)
{
    ltc_itc();
    EXTI_ClearITPendingBit(EXTI_Line6);
}
#endif