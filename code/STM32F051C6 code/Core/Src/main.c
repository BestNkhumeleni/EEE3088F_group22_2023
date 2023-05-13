#include "stm32f0xx.h"
#include "lcd_stm32f0.h"

#define LED_PIN_1 GPIO_PIN_7
#define LED_PORT_1 GPIOB
#define LED_PIN_2 GPIO_PIN_6
#define LED_PORT_2 GPIOB
#define LED_PIN_3 GPIO_PIN_5
#define LED_PORT_3 GPIOB
#define LED_PIN_4 GPIO_PIN_4
#define LED_PORT_4 GPIOB
#define LED_PIN_5 GPIO_PIN_3
#define LED_PORT_5 GPIOB
#define LED_PIN_6 GPIO_PIN_2
#define LED_PORT_6 GPIOB
#define LED_PIN_7 GPIO_PIN_1
#define LED_PORT_7 GPIOB
#define LED_PIN_8 GPIO_PIN_0
#define LED_PORT_8 GPIOB



#define BUTTON_PIN GPIO_PIN_0
#define BUTTON_PORT GPIOA

void SystemClock_Config(void);
void GPIO_Init(void);
void delay_ms(uint32_t milliseconds);

int main(void)
{
  SystemInit();
  SystemClock_Config();
  GPIO_Init();

  uint8_t button_press_count = 0;

  while (1)
  {
    if (!(GPIOA->IDR & BUTTON_PIN)) // Check if SW0 button is pressed
    {
      delay_ms(10); // Delay for de-bouncing

      if (!(GPIOA->IDR & BUTTON_PIN)) // Check if SW0 button is still pressed after debouncing
      {
        button_press_count++;

        if (button_press_count == 1)
        {
          GPIOB->BSRR = LED_PIN_1; // Turn on LED1
        }
        else if (button_press_count == 2)
        {
          GPIOB->BSRR = LED_PIN_2; // Turn on LED2
        }
        else if (button_press_count ==3){
        	 GPIOB->BSRR = LED_PIN_3; // Turn on LED3
        }
        if (button_press_count == 4)
        {
          GPIOB->BSRR = LED_PIN_4; // Turn on LED1
        }
        else if (button_press_count == 5)
        {
          GPIOB->BSRR = LED_PIN_5; // Turn on LED2
        }
        else if (button_press_count == 6){
        	 GPIOB->BSRR = LED_PIN_6; // Turn on LED3
        }
        else if (button_press_count == 7){
                	 GPIOB->BSRR = LED_PIN_7; // Turn on LED3
                }
        else if (button_press_count == 8){
                	 GPIOB->BSRR = LED_PIN_8; // Turn on LED3
                }

        else if (button_press_count == 9)
        {
          GPIOB->BRR = LED_PIN_1;
          GPIOB->BRR = LED_PIN_2;
          GPIOB->BRR = LED_PIN_3;
          GPIOB->BRR = LED_PIN_4;
          GPIOB->BRR = LED_PIN_5;
          GPIOB->BRR = LED_PIN_6;
          GPIOB->BRR = LED_PIN_7;
          GPIOB->BRR = LED_PIN_8;
          button_press_count = 0; // Reset the counter
        }
      }

      // Wait until the button is released
      while (!(GPIOA->IDR & BUTTON_PIN))
      {
        // Optional: Add additional logic here if needed
      }
    }
  }
}

void SystemClock_Config(void)
{
  RCC->CR |= RCC_CR_HSION; // Enable HSI clock
  while (!(RCC->CR & RCC_CR_HSIRDY)); // Wait until HSI is ready

  RCC->CFGR |= RCC_CFGR_SW_HSI; // Select HSI as the system clock
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI); // Wait until HSI is used as the system clock
}

void GPIO_Init(void)
{
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN; // Enable GPIOA and GPIOB clocks

  GPIOB->MODER |= GPIO_MODER_MODER7_0 | GPIO_MODER_MODER6_0 | GPIO_MODER_MODER5_0 | GPIO_MODER_MODER4_0 | GPIO_MODER_MODER3_0 | GPIO_MODER_MODER2_0 | GPIO_MODER_MODER1_0 | GPIO_MODER_MODER0_0; // Set LED pins as output
  GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_7 | GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_5 | GPIO_OTYPER_OT_4 | GPIO_OTYPER_OT_3 | GPIO_OTYPER_OT_2 | GPIO_OTYPER_OT_1 | GPIO_OTYPER_OT_0); // Set LED pins as push-pull
  GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR7 | GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR5 | GPIO_OSPEEDER_OSPEEDR4 | GPIO_OSPEEDER_OSPEEDR3 | GPIO_OSPEEDER_OSPEEDR2 | GPIO_OSPEEDER_OSPEEDR1 | GPIO_OSPEEDER_OSPEEDR0; // Set LED pins speed to high
  GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR7 | GPIO_PUPDR_PUPDR6 | GPIO_PUPDR_PUPDR5 | GPIO_PUPDR_PUPDR4 | GPIO_PUPDR_PUPDR3 | GPIO_PUPDR_PUPDR2 | GPIO_PUPDR_PUPDR1 | GPIO_PUPDR_PUPDR0); // Disable pull-up/down resistors

  GPIOA->MODER &= ~GPIO_MODER_MODER0; // Set SW0 button pin as input
  GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_0; // Enable pull-up resistor for SW0 button pin
}

void delay_ms(uint32_t milliseconds)
{
  volatile uint32_t i;
  for (i = 0; i < (milliseconds * 800); i++){} // Rough delay approximation
}
