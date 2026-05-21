# dsPIC-FreeRTOS-Hatch-Controller
A real-time hatch control system powered by FreeRTOS and a dsPIC33FJ128MC802 MCU. It uses a servo to adjust a hatch based on temperature readings from a DS18B20 sensor. It features Auto/Manual modes, an LCD, and a UART interface for PC control.
----------------------------------
Developed a real-time embedded system that automatically controls a mechanical hatch based on ambient temperature.

-> Interfaced a DS18B20 digital temperature sensor and driven a servomotor via PWM to physically adjust the hatch.
-> Implemented an Auto mode (adjusting the servo proportionally for temperatures between 20°C and 30°C) and a Manual mode (controlled via a potentiometer and ADC).
-> Designed a UART serial interface for PC communication and command processing, alongside a 4-bit LCD for local status monitoring.
-> Successfully managed concurrent FreeRTOS tasks including sensor reading, PWM scaling, ADC conversion, hardware interrupt debouncing, and serial dialogue.
