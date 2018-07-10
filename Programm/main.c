#include <stdint.h>
#include "stm32f10x.h"
#include "ws2812b.h"

void HSV_to_RGB(int hue, int sat, int val, uint8_t *rc, uint8_t *gc, uint8_t *bc) ;

void main(void)
{
  uint8_t rc;
  uint8_t gc;
  uint8_t bc;
  
  ws2812b_init();
  while(!ws2812b_is_ready())
    ;
  
  
  /*
  
  /// Устанавливаем всем светодиодам 
  /// красный цвет свечения
  
  int i;
  for(i=0; i<WS2812B_NUM_LEDS; i++)
  {
    ws2812b_set(i, 100, 0, 0);
  }
  
  ws2812b_send();
  
  */
  
  
  /// Эффект "Мерцающая радуга"
  
  int k;
  
  for(;;)
  {  
    for(k=0; k < 255; k++) //Изменяем яркость от 0 до 255
    {
      //Заполняем буфер светодиодной ленты радугой с яркостью k
      for(int i=0; i<WS2812B_NUM_LEDS; i++) 
      {
        HSV_to_RGB((int)(i*360/WS2812B_NUM_LEDS), 255, k, &rc, &gc, &bc);
        ws2812b_set(i, rc, gc, bc);
      }
      
      //Выводим буфер
      ws2812b_send();
      
      //Ждем окончания передачи
      while(!ws2812b_is_ready())
        ;
    }
    
    for(; k >= 0; k--) //Изменяем яркость от 255 до 0
    {
      for(int i=0; i<WS2812B_NUM_LEDS; i++)
      {
        HSV_to_RGB((int)(i*360/WS2812B_NUM_LEDS), 255, k, &rc, &gc, &bc);
        ws2812b_set(i, rc, gc, bc);
      }
      
      ws2812b_send();
      
      while(!ws2812b_is_ready())
        ;
    }
  
  }
  
}


/// Корнвертер из HSV в RGB 
/// Взял отсюда:
/// Статья https://habr.com/post/257131/
/// Ссылка https://drive.google.com/file/d/0B5dbvc_yPqJHQ2FEUXpkR3NocnM/view
/// Оригинальный алгоритм немного переделан, а именно
///  изменен механизм возврата значения компонент R, G, B

const uint8_t dim_curve[256] = {
  0, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3,
  3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4,
  4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6,
  6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8,
  8, 8, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 11, 11, 11,
  11, 11, 12, 12, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 15,
  15, 15, 16, 16, 16, 16, 17, 17, 17, 18, 18, 18, 19, 19, 19, 20,
  20, 20, 21, 21, 22, 22, 22, 23, 23, 24, 24, 25, 25, 25, 26, 26,
  27, 27, 28, 28, 29, 29, 30, 30, 31, 32, 32, 33, 33, 34, 35, 35,
  36, 36, 37, 38, 38, 39, 40, 40, 41, 42, 43, 43, 44, 45, 46, 47,
  48, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62,
  63, 64, 65, 66, 68, 69, 70, 71, 73, 74, 75, 76, 78, 79, 81, 82,
  83, 85, 86, 88, 90, 91, 93, 94, 96, 98, 99, 101, 103, 105, 107, 109,
  110, 112, 114, 116, 118, 121, 123, 125, 127, 129, 132, 134, 136, 139, 141, 144,
  146, 149, 151, 154, 157, 159, 162, 165, 168, 171, 174, 177, 180, 183, 186, 190,
  193, 196, 200, 203, 207, 211, 214, 218, 222, 226, 230, 234, 238, 242, 248, 255,
};

/*------------------------------------------------------------------------------
  Корнвертер из HSV в RGB в целочисленной арифмерите
 
  hue        : 0..360
  saturation : 0..255
  value      : 0..255
 ------------------------------------------------------------------------------*/
void HSV_to_RGB(int hue, int sat, int val, uint8_t *rc, uint8_t *gc, uint8_t *bc) 
{
  int    r;
  int    g;
  int    b;
  int    base;
  //uint32_t rgb;

  val = dim_curve[val];
  sat = 255 - dim_curve[255 - sat];


  if ( sat == 0 ) // Acromatic color (gray). Hue doesn't mind.
  {
    (*rc) = val;
    (*gc) = val;
    (*bc) = val;
    //rgb = val | (val<<8) | (val <<16);
  }
  else
  {
    base = ((255 - sat) * val) >> 8;
    switch (hue / 60)
    {
    case 0:
      r = val;
      g = (((val - base) * hue) / 60) + base;
      b = base;
      break;
    case 1:
      r = (((val - base) * (60 - (hue % 60))) / 60) + base;
      g = val;
      b = base;
      break;
    case 2:
      r = base;
      g = val;
      b = (((val - base) * (hue % 60)) / 60) + base;
      break;
    case 3:
      r = base;
      g = (((val - base) * (60 - (hue % 60))) / 60) + base;
      b = val;
      break;
    case 4:
      r = (((val - base) * (hue % 60)) / 60) + base;
      g = base;
      b = val;
      break;
    case 5:
      r = val;
      g = base;
      b = (((val - base) * (60 - (hue % 60))) / 60) + base;
      break;
    }
    (*rc) = r & 0xFF;
    (*gc) = g & 0xFF;
    (*bc) = b & 0xFF;
    //rgb = ((r & 0xFF)<<16) | ((g & 0xFF)<<8) | (b & 0xFF);
  }
  //return rgb;
}
