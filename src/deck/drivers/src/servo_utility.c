#include "motors.h"

static uint16_t act_max = 65535;
uint16_t servo_ratio = 0; 
bool isInit = false;

void servoInit(MotorPerifDef* servoDef) {
    PIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    MOTORS_RCC_GPIO_CMD(servoDef->gpioPerif, ENABLE);
    MOTORS_RCC_GPIO_CMD(servoDef->gpioPowerswitchPerif, ENABLE);
    MOTORS_RCC_TIM_CMD(servoDef->timPerif, ENABLE);

    if (servoDef->gpioPowerswitchPin != 0)
    {
        GPIO_StructInit(&GPIO_InitStructure);
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Pin = servoDef->gpioPowerswitchPin;
        GPIO_Init(servoDef->gpioPowerswitchPort, &GPIO_InitStructure);
        GPIO_WriteBit(servoDef->gpioPowerswitchPort, servoDef->gpioPowerswitchPin, 1);
    }

    // Configure the GPIO for the timer output
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode = MOTORS_GPIO_MODE;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = servoDef->gpioOType;
    GPIO_InitStructure.GPIO_Pin = servoDef->gpioPin;
    GPIO_Init(servoDef->gpioPort, &GPIO_InitStructure);

    //Map timers to alternate functions
    MOTORS_GPIO_AF_CFG(servoDef->gpioPort, servoDef->gpioPinSource, servoDef->gpioAF);

    //Timer configuration
    TIM_TimeBaseStructure.TIM_Period = servoDef->timPeriod;
    TIM_TimeBaseStructure.TIM_Prescaler = servoDef->timPrescaler;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(servoDef->tim, &TIM_TimeBaseStructure);

    // PWM channels configuration (All identical!)
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = servoDef->timPolarity;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

    // Configure Output Compare for PWM
    servoDef->ocInit(servoDef->tim, &TIM_OCInitStructure);
    servoDef->preloadConfig(servoDef->tim, TIM_OCPreload_Enable);

    // Start the timer
    TIM_Cmd(servoDef->tim, ENABLE);

    isInit = true;

    // Output zero power
    servoStop();
}

void servoDeInit(const MotorPerifDef** motorMapSelect)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // Configure default
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = servoDef->gpioPin;
    GPIO_Init(servoDef->gpioPort, &GPIO_InitStructure);

    //Map timers to alternate functions
    GPIO_PinAFConfig(servoDef->gpioPort, servoDef->gpioPinSource, 0x00);

    //Deinit timer
    TIM_DeInit(servoDef->tim);
}

bool servoTest(void)
{
    servoSetRatio(70);
    return isInit;
}
void servoSetRatio(uint16_t driveAngle)
{
  if (isInit) {

    uint16_t ratio = driveAngle*act_max/100.0f;

    servo_ratio = ratio;

    servoDef->setCompare(servoDef->tim, servoBLConv16ToBits(ratio));

    /*  uint64_t currTime = usecTimestamp();
      cycleTime = currTime - lastCycleTime;
      lastCycleTime = currTime;
      */
    }
}

void servoEnablePWM(void)
{
    TIM_CtrlPWMOutputs(servoDef->tim, ENABLE);
}

void servoDisablePWM(void)
{
    TIM_CtrlPWMOutputs(servoDef->tim, DISABLE);
}

void servoStop()
{
    servoSetRatio(0);
}
static uint16_t servoBLConv16ToBits(uint16_t bits)
{
  return (MOTORS_BL_PWM_CNT_FOR_HIGH + ((bits * MOTORS_BL_PWM_CNT_FOR_HIGH) / 0xFFFF));
}