# GSM Communication System
This repository contains GSM communication system project

# Description

This project is using SIM800L v2 GSM module to provide communication between microcontroller and a mobile phone.
Electronic analog temperature sensor monitors temperature. If it's exceeded, microcontroller sends SMS message to selected mobile phone with help of GSM module. If temperature is back in safe range, it also sends message. User can control state of selected pin of mirocontroller by sending SMS commands. Received messages are displaying on LCD 2x16 display.

SIM800L v2 module is controlled by using AT commands sent via UART. Whole system is based on interrupts. 

## Getting Started

This project is customized to use with STM32F446RE module. 
Elements used in this project:
-STM32F446RE board
-LM35DZ temperature sensor
-LCD 2x16
-LCD I2C converter
-SIM800L v2 GSM module
-Transoptors to equalize voltage levels

This project uses HAL libraries, and Mohamed Yaqoob's LCD library 

To get knowlage about used pins and peripherials check "Pinout" file. 

## Authors

* **Marek Jaromin** - *Initial work* 
