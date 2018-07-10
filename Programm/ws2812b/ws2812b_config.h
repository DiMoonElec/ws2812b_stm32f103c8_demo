/******************************************************************************
File:   ws2812b_config.h
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


#ifndef __WS2812B_CONF_H__
#define __WS2812B_CONF_H__

#include <stdint.h>


//Период следования бит в тиках таймера
//должно быть 1.25мкс
#define WS2812B_TIMER_AAR       0x0059

//Передача лог. нуля 0.4мкс
#define WS2812B_0_VAL           (WS2812B_TIMER_AAR / 3)

//Передача лог. единицы 0.85мкс
#define WS2812B_1_VAL           ((WS2812B_TIMER_AAR / 3) * 2)

//Сигнал RET или RESET более 50мкс
#define WS2812B_TIMER_RET       (WS2812B_TIMER_AAR * 45)

//убрать коментарий, если нужно инвертировать 
//выходной сигнал
  #define WS2812B_OUTPUT_INVERSE

//Какой вывод использовать для формирования сигнала
/*
Возможные варианты
 Знач.  Порт
 0      PA0
 1      PA1
 2      PA2
 3      PA3
*/
#define WS2812B_OUTPUT_PAx      1








#endif
