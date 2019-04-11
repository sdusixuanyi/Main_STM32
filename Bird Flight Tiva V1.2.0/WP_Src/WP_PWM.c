#include "Headfile.h" 
#include "WP_PWM.h"
#define PWM_Period_MAX  3125//2.5ms¡ª¡ª¡ª¡ª400hz
static uint16_t period;
void Init_PWM(void)//PWM³õÊ¼»¯
{
  SysCtlPWMClockSet(SYSCTL_PWMDIV_64); // Set divider to 80M/8=10M=0.1us
  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0); // Enable PWM peripheral
  SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); // Enable GPIOB peripheral
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC); // Enable GPIOC peripheral
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); // Enable GPIOE peripheral
  SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated
  // Use alternate function
  GPIOPinConfigure(GPIO_PB6_M0PWM0);
  GPIOPinConfigure(GPIO_PB7_M0PWM1);
  GPIOPinConfigure(GPIO_PB4_M0PWM2);
  GPIOPinConfigure(GPIO_PB5_M0PWM3);
  
  GPIOPinConfigure(GPIO_PE4_M0PWM4);
  GPIOPinConfigure(GPIO_PE5_M0PWM5);
  GPIOPinConfigure(GPIO_PC4_M0PWM6);
  GPIOPinConfigure(GPIO_PC5_M0PWM7);
  // Use pin with PWM peripheral
  GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);//M0PWM0
  GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7);//M0PWM1
  GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4);//M0PWM2
  GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_5);//M0PWM3
  
  GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4);//M0PWM4
  GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_5);//M0PWM5
  GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_4);//M0PWM6
  GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_5);//M0PWM7
  // Configure the PWM generator for count down mode with immediate updates to the parameters
  PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
  PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
  
  PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
  PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
  // The period is set to 2.5ms (400 Hz)
  period = PWM_Period_MAX; 
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, period); // Set the period
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, period);
  
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, period); // Set the period
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, period);
  // Start the timers in generator 0 and 1
  PWMGenEnable(PWM0_BASE, PWM_GEN_0);
  PWMGenEnable(PWM0_BASE, PWM_GEN_1);
  
  PWMGenEnable(PWM0_BASE, PWM_GEN_2);
  PWMGenEnable(PWM0_BASE, PWM_GEN_3);
  // Enable the outputs
  PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT | PWM_OUT_2_BIT | PWM_OUT_3_BIT
                 | PWM_OUT_4_BIT | PWM_OUT_5_BIT | PWM_OUT_6_BIT | PWM_OUT_7_BIT, true);
  PWM_Output(1250,1250,1250,1250,1250,1250,1250,1250);
}



void PWM_Output(uint16_t width1,uint16_t width2,uint16_t width3,uint16_t width4,
                uint16_t width5,uint16_t width6,uint16_t width7,uint16_t width8)
{
  PWMPulseWidthSet(PWM0_BASE,PWM_OUT_6,width1);//PC4 
  PWMPulseWidthSet(PWM0_BASE,PWM_OUT_7,width2);//PC5  
  PWMPulseWidthSet(PWM0_BASE,PWM_OUT_1,width3);//PB7
  PWMPulseWidthSet(PWM0_BASE,PWM_OUT_0,width4);//PB6
  PWMPulseWidthSet(PWM0_BASE,PWM_OUT_5,width5);//PE5
  PWMPulseWidthSet(PWM0_BASE,PWM_OUT_4,width6);//P43
  PWMPulseWidthSet(PWM0_BASE,PWM_OUT_2,width7);//PB4
  PWMPulseWidthSet(PWM0_BASE,PWM_OUT_3,width8);//PB5
}


