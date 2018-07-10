/******************************************************************************
File:   ws2812b.c
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



#include "stm32f10x.h"
#include "ws2812b.h"
#include "ws2812b_config.h"

/*****************************************************************************/

#if (WS2812B_OUTPUT_PAx==0)

  #define GPIO_CRL_CNFx           GPIO_CRL_CNF0
  #define GPIO_CRL_CNFx_1         GPIO_CRL_CNF0_1
  #define GPIO_CRL_MODEx_1        GPIO_CRL_MODE0_1
  #define GPIO_CRL_MODEx_0        GPIO_CRL_MODE0_0

  #define TIM_CCER_CCxE           TIM_CCER_CC1E
  #define TIM_CCER_CCxP           TIM_CCER_CC1P

  #define CCMRx                   CCMR1
  #define TIM_CCMRy_OCxM          TIM_CCMR1_OC1M
  #define TIM_CCMRy_OCxM_2        TIM_CCMR1_OC1M_2
  #define TIM_CCMRy_OCxM_1        TIM_CCMR1_OC1M_1
  #define TIM_CCMRy_OCxPE         TIM_CCMR1_OC1PE
  #define TIM_DIER_CCxDE          TIM_DIER_CC1DE
  #define CCRx                    CCR1

  #define DMA1_Channelx           DMA1_Channel5
  #define DMA1_Channelx_IRQn      DMA1_Channel5_IRQn
  #define DMA1_Channelx_IRQHandler        DMA1_Channel5_IRQHandler

  #define DMA_CCRx_EN             DMA_CCR5_EN
  #define DMA_CCRx_TCIE           DMA_CCR5_TCIE

  #define DMA_IFCR_CTEIFx         DMA_IFCR_CTEIF5
  #define DMA_IFCR_CHTIFx         DMA_IFCR_CHTIF5
  #define DMA_IFCR_CTCIFx         DMA_IFCR_CTCIF5
  #define DMA_IFCR_CGIFx          DMA_IFCR_CGIF5

#elif (WS2812B_OUTPUT_PAx==1)

  #define GPIO_CRL_CNFx           GPIO_CRL_CNF1
  #define GPIO_CRL_CNFx_1         GPIO_CRL_CNF1_1
  #define GPIO_CRL_MODEx_1        GPIO_CRL_MODE1_1
  #define GPIO_CRL_MODEx_0        GPIO_CRL_MODE1_0

  #define TIM_CCER_CCxE           TIM_CCER_CC2E
  #define TIM_CCER_CCxP           TIM_CCER_CC2P

  #define CCMRx                   CCMR1
  #define TIM_CCMRy_OCxM          TIM_CCMR1_OC2M
  #define TIM_CCMRy_OCxM_2        TIM_CCMR1_OC2M_2
  #define TIM_CCMRy_OCxM_1        TIM_CCMR1_OC2M_1
  #define TIM_CCMRy_OCxPE         TIM_CCMR1_OC2PE
  #define TIM_DIER_CCxDE          TIM_DIER_CC2DE
  #define CCRx                    CCR2

  #define DMA1_Channelx           DMA1_Channel7
  #define DMA1_Channelx_IRQn      DMA1_Channel7_IRQn
  #define DMA1_Channelx_IRQHandler        DMA1_Channel7_IRQHandler

  #define DMA_CCRx_EN             DMA_CCR7_EN
  #define DMA_CCRx_TCIE           DMA_CCR7_TCIE

  #define DMA_IFCR_CTEIFx         DMA_IFCR_CTEIF7
  #define DMA_IFCR_CHTIFx         DMA_IFCR_CHTIF7
  #define DMA_IFCR_CTCIFx         DMA_IFCR_CTCIF7
  #define DMA_IFCR_CGIFx          DMA_IFCR_CGIF7


#elif (WS2812B_OUTPUT_PAx==2)

  #define GPIO_CRL_CNFx           GPIO_CRL_CNF2
  #define GPIO_CRL_CNFx_1         GPIO_CRL_CNF2_1
  #define GPIO_CRL_MODEx_1        GPIO_CRL_MODE2_1
  #define GPIO_CRL_MODEx_0        GPIO_CRL_MODE2_0

  #define TIM_CCER_CCxE           TIM_CCER_CC3E
  #define TIM_CCER_CCxP           TIM_CCER_CC3P

  #define CCMRx                   CCMR2
  #define TIM_CCMRy_OCxM          TIM_CCMR2_OC3M
  #define TIM_CCMRy_OCxM_2        TIM_CCMR2_OC3M_2
  #define TIM_CCMRy_OCxM_1        TIM_CCMR2_OC3M_1
  #define TIM_CCMRy_OCxPE         TIM_CCMR2_OC3PE
  #define TIM_DIER_CCxDE          TIM_DIER_CC3DE
  #define CCRx                    CCR3

  #define DMA1_Channelx           DMA1_Channel1
  #define DMA1_Channelx_IRQn      DMA1_Channel1_IRQn
  #define DMA1_Channelx_IRQHandler        DMA1_Channel1_IRQHandler

  #define DMA_CCRx_EN             DMA_CCR1_EN
  #define DMA_CCRx_TCIE           DMA_CCR1_TCIE

  #define DMA_IFCR_CTEIFx         DMA_IFCR_CTEIF1
  #define DMA_IFCR_CHTIFx         DMA_IFCR_CHTIF1
  #define DMA_IFCR_CTCIFx         DMA_IFCR_CTCIF1
  #define DMA_IFCR_CGIFx          DMA_IFCR_CGIF1

#elif (WS2812B_OUTPUT_PAx==3)

#define GPIO_CRL_CNFx           GPIO_CRL_CNF3
#define GPIO_CRL_CNFx_1         GPIO_CRL_CNF3_1
#define GPIO_CRL_MODEx_1        GPIO_CRL_MODE3_1
#define GPIO_CRL_MODEx_0        GPIO_CRL_MODE3_0

#define TIM_CCER_CCxE           TIM_CCER_CC4E
#define TIM_CCER_CCxP           TIM_CCER_CC4P

#define CCMRx                   CCMR2
#define TIM_CCMRy_OCxM          TIM_CCMR2_OC4M
#define TIM_CCMRy_OCxM_2        TIM_CCMR2_OC4M_2
#define TIM_CCMRy_OCxM_1        TIM_CCMR2_OC4M_1
#define TIM_CCMRy_OCxPE         TIM_CCMR2_OC4PE
#define TIM_DIER_CCxDE          TIM_DIER_CC4DE
#define CCRx                    CCR4

#define DMA1_Channelx           DMA1_Channel7
#define DMA1_Channelx_IRQn      DMA1_Channel7_IRQn
#define DMA1_Channelx_IRQHandler        DMA1_Channel7_IRQHandler

#define DMA_CCRx_EN             DMA_CCR7_EN
#define DMA_CCRx_TCIE           DMA_CCR7_TCIE

#define DMA_IFCR_CTEIFx         DMA_IFCR_CTEIF7
#define DMA_IFCR_CHTIFx         DMA_IFCR_CHTIF7
#define DMA_IFCR_CTCIFx         DMA_IFCR_CTCIF7
#define DMA_IFCR_CGIFx          DMA_IFCR_CGIF7

#endif

//Расчитываем длину буфера
#define DATA_LEN ((WS2812B_NUM_LEDS * 24) + 2)

static uint8_t led_array[DATA_LEN];
static int flag_rdy = 0;

static void bus_retcode(void);

void ws2812b_init(void)
{
  flag_rdy = 0;
  
  //Разрешаем такирование переферии
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; //Включаем тактирование порта GPIOA
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; //таймера TIM2
  RCC->AHBENR |= RCC_AHBENR_DMA1EN;   //и DMA1
  
  /********* Настраиваем PA1 *********/
  //PA1 freq=10Mhz, AF output Push-pull
  GPIOA->CRL &= ~(GPIO_CRL_CNFx);
  GPIOA->CRL |= GPIO_CRL_CNFx_1 
    | GPIO_CRL_MODEx_1 | GPIO_CRL_MODEx_0;
  
  
  /********* Настойка таймера TIM2 *********/
  //Разрешаем таймеру управлять выводом PA1
  TIM2->CCER |= TIM_CCER_CCxE;    //Разрешаем
  
#ifdef WS2812B_OUTPUT_INVERSE
  TIM2->CCER |= TIM_CCER_CCxP;    //Вывод инверсный
#else
  TIM2->CCER &= ~(TIM_CCER_CCxP); //Вывод не инверсный
#endif
  
  
  TIM2->CCMRx &= ~(TIM_CCMRy_OCxM); //сбрасываем все биты OCxM
  
  //устанавливаем выход в неактивное состояние
  TIM2->CCMRx |= TIM_CCMRy_OCxM_2; 
  TIM2->CCMRx &= ~(TIM_CCMRy_OCxM_2);
    
  TIM2->CCMRx |= TIM_CCMRy_OCxM_2 | TIM_CCMRy_OCxM_1 
    | TIM_CCMRy_OCxPE; //режим ШИМ-а
  
  TIM2->CR1 |= TIM_CR1_ARPE;    //Регистры таймера с буферизацией
  TIM2->DIER |= TIM_DIER_CCxDE; //Разрешить запрос DMA
  
  //Настраиваем канал DMA
  DMA1_Channelx->CPAR = (uint32_t)(&TIM2->CCRx); //Куда пишем
  DMA1_Channelx->CMAR = (uint32_t)(led_array); //откуда берем
  
  DMA1_Channelx->CCR = DMA_CCR7_PSIZE_0 //регистр переферии 16 бит
    | DMA_CCR7_MINC //режим инкремента указателя памяти
    | DMA_CCR7_DIR; //напревление передачи из памяти в переферию
  
  //Разрешаем обработку прерываний
  NVIC_EnableIRQ(TIM2_IRQn); //от таймера
  NVIC_EnableIRQ(DMA1_Channelx_IRQn); //от DMA
  
  ws2812b_buff_claer();
  bus_retcode(); //сбрасываем шину
}

void ws2812b_buff_claer(void)
{
  for(int i = 0; i<WS2812B_NUM_LEDS-2; i++)
    led_array[i] = WS2812B_0_VAL;
  
  led_array[DATA_LEN-2] = 0;
  led_array[DATA_LEN-1] = 0;
}

int ws2812b_set(int pixn, uint8_t r, uint8_t g, uint8_t b)
{
  int offset = pixn*24;
  int i;
  uint8_t tmp;
  
  if(pixn > (WS2812B_NUM_LEDS - 1))
    return 1;
  
  //g component
  tmp = g;
  for(i=0; i<8; i++)
  {
    if(tmp & 0x80)
      led_array[offset + i] = WS2812B_1_VAL;
    else
      led_array[offset + i] = WS2812B_0_VAL;
    tmp<<=1;
  }
  
  //r component
  tmp = r;
  for(i=0; i<8; i++)
  {
    if(tmp & 0x80)
      led_array[offset + i + 8] = WS2812B_1_VAL;
    else
      led_array[offset + i + 8] = WS2812B_0_VAL;
    tmp<<=1;
  }
  
  //b component
  tmp = b;
  for(i=0; i<8; i++)
  {
    if(tmp & 0x80)
      led_array[offset + i + 16] = WS2812B_1_VAL;
    else
      led_array[offset + i + 16] = WS2812B_0_VAL;
    tmp<<=1;
  }
  
  return 0;
}

int ws2812b_send(void)
{
  if(flag_rdy) //Если сейчас ни чего не передается
  {
    //Устанавливаем флаг занятости интерфейса
    flag_rdy = 0;
    
    //Настраиваем передачу данных
    DMA1_Channelx->CCR &= ~(DMA_CCR7_EN); //Отключаем канал DMA
    DMA1_Channelx->CNDTR = sizeof(led_array); //Устанавливаем количество данных
    
    //Таймер считает до WS2812B_TIMER_AAR, таким образом
    //при данной частоте тактирования таймера
    //получаем период ШИМ-сигнала, равный 1.25мкс
    TIM2->ARR = WS2812B_TIMER_AAR;
    TIM2->CCRx = 0x0000; //Устанавливаем ШИМ-регистр таймера в ноль
    TIM2->CNT = 0; //Очищаем счетный регистр
    TIM2->CR1 |= TIM_CR1_CEN; //Запускаем таймер
    //Так как значение ШИМ установили в ноль, 
    //то на шине будет установлен неактивный уровень
    //до момента запуска DMA  
    
    DMA1->IFCR = DMA_IFCR_CTEIFx | DMA_IFCR_CHTIFx 
      | DMA_IFCR_CTCIFx | DMA_IFCR_CGIFx; //Очищаем все флаги прерываний DMA
    
    DMA1_Channelx->CCR |= DMA_CCRx_TCIE; //прерывание завершения передачи
    
    //Включаем канал DMA, тем самым начинаем передачу данных
    DMA1_Channelx->CCR |= DMA_CCRx_EN; 
    return 0;
  }
  else
  {
    return 1;
  }
}


int ws2812b_is_ready(void)
{
  return flag_rdy;
}

static void bus_retcode(void)
{
  TIM2->CR1 &= ~(TIM_CR1_CEN); //останавливаем таймер
  TIM2->ARR = WS2812B_TIMER_RET; //Устанавливаем период немного больше 50мкс
  TIM2->CNT = 0; //Очищаем счетный регистр
  TIM2->CCRx = 0x0000; //значение ШИМ-а ноль
  TIM2->SR &= ~(TIM_SR_UIF); //сбрасываем флаг прерывания
  TIM2->DIER |= TIM_DIER_UIE; //прерывание по обновлению
  TIM2->CR1 |= TIM_CR1_CEN; //Поехали считать!
}

//Прерывание от DMA
//Суда попадаем после завершения передачи данных
void DMA1_Channelx_IRQHandler(void)
{
  DMA1_Channelx->CCR &= ~(DMA_CCRx_EN); //Отключаем канал DMA
  
  DMA1->IFCR = DMA_IFCR_CTEIFx | DMA_IFCR_CHTIFx 
    | DMA_IFCR_CTCIFx | DMA_IFCR_CGIFx; //Сбрасываем все флаги прерываний
  
  //Так как последние 2 элемента массива равны нулю,
  //то сейчас предпоследнее значение уже загружено
  //в теневой регистр сравнения
  //и на шине установлено неактивное состояние.
  //Задача заключается в удержании шины в этом состоянии
  //в течение 50мкс или более
  //перед установкой флага готовности интерфейса.
  
  bus_retcode();
}


//прерывание от таймера
//Сюда попадаем после завершения формирования 
//сигнала RET шины ws2812b
void TIM2_IRQHandler(void)
{
  TIM2->SR = 0; //Сбрасываем все флаги прерываний
  
  //Итак, мы завершили формирование сигнала RET на шине
  //и теперь можно сделать все завершающие операции 
  //и установить флаг готовности интерфейса к следующей
  //передаче данных.
  
  TIM2->CR1 &= ~(TIM_CR1_CEN); //останавливаем таймер
  TIM2->DIER &= ~(TIM_DIER_UIE); //запрещаем прерывание таймера
  
  flag_rdy = 1;
}