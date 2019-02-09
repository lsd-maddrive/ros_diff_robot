# Использование модулей периферии контроллера

## STM32F767ZI подключение

### Управление двигателями

STM pin | Shield pin | Description
--------|------------|------------
PB_10   | D6		 | PWM2/3 (Inv) - RMotor+
PB_11   | D5		 | PWM2/4 (Inv) - RMotor-
PD_12   | D11		 | PWM4/1 (Inv) - LMotor+
PD_13   | D3		 | PWM4/2 (Inv) - LMotor-

### Энкодеры

STM pin | Shield pin | Description
--------|------------|------------
PE_14   | D13		 | REnoder Green  / A
PE_15   | D12		 | REnoder Orange / B
PE_12   | 3V3		 | LEnoder Green  / B
PE_10   | AREF		 | LEnoder Orange / A

> В энкодере правом и левом каналы на разных цветах из-за формулировки прошивки, подробнее см. [модуль](encoders.c#L8).

## [Arduino nano shield](https://www.elechouse.com/elechouse/images/product/wifi%20car%20driver/) подключение

### Энкодер Правый

Source wire / Color | Shield pin
--------------------|------------
Yellow				| GND
White				| +5V
Green				| D13
Orange				| D12

### Энкодер Левый

Source wire / Color | Shield pin
--------------------|------------
Yellow				| GND
White				| +5V
Green				| 3V3
Orange				| AREF
