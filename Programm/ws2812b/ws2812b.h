/******************************************************************************
File:   ws2812b.h
Ver     1.0
Date:   July 10, 2018
Autor:  Sivokon Dmitriy aka DiMoon Electronics

*******************************************************************************
BSD 2-Clause License

Copyright (c) 2018, Sivokon Dmitriy
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

******************************************************************************/


#ifndef __WS2812B_H__
#define __WS2812B_H__

#include <stdint.h>

//Количество светодиодов в ленте
#define WS2812B_NUM_LEDS        144

//Инициализация интерфейса ws2812b
void ws2812b_init(void);

//Очистить буфер светодиодной ленты.
//Устанавливает всем светодиодам значения
//R=0, G=0, B=0
void ws2812b_buff_claer(void);

//Установить компоненты RGB светодиода номер pixn
//pixn=0..WS2812B_NUM_LEDS-1
//r=0..255, g=0..255, b=0..255
//Возвращаемые значения
// 0 - выполнено успешно
// 1 - неверное значение pixn
int ws2812b_set(int pixn, uint8_t r, uint8_t g, uint8_t b);

//Загрузить подготовленный буфрер 
//в светодиодную ленту.
//Возврашает 1 если предыдущая операция 
//обмена данными еще не завершена
int ws2812b_send(void);

//Возвращает 1 если предыдущая операция 
//обмена данными с светодиодной лентой
//завершена успешно
int ws2812b_is_ready(void);


#endif

