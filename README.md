# STM32_TEMPERATURE_SENSOR

TEMPERATURE and HUMIDITY SENSOR with using STM32F0

=======================================================

.ioc

Clock Configuration

- Set HCLK(MHz) as 48 

Pinout & Configuration

SYSTEM CORE
SYS
- Enable "Debug Serial Wire"
- Timebase Source : SysTick
RCC
- HSE : Crystal / Ceramic Resonator

Timers
- Clock Source : Internal Clock
- Prescaler : 47 (48-1)

Connectivity (Optional for LCD)
I2C1
- I2C : I2C
(Automatically PB7 and PB6 pins are selected as I2C2_SDA and I2C1_SCL respectively) 
- Parameter Settings > I2C Speed Mode : Fast Mode

- Set PB3, PC12 and PC10 pins as GPIO_Output for LEDs
- Set PB9 as GPIO_Output for sensor

=======================================================

For Liquid-crystal display TC1602A (Optional for LCD)
Add i2c-lcd.h and i2c-lcd.c libraries into Inc and Src for displaying.
I used this link https://controllerstech.com/i2c-lcd-in-stm32/ for getting libraries.

GND : GND 
VCC : 5V
SDA : PB7
SCL : PB6

=======================================================

For LEDs

GREEN  LED (+) : PB3
RED    LED (+) : PC12
YELLOW LED (+) : PC10

=======================================================

For Temperature Sensor
I used this link https://www.micropeta.com/video39 for getting temperature sensor code

(+) : 3V
(-) : GND
OUT : PB9

=======================================================

