# w5500_lib_stm32F411
w5500 간단한 라이브러리  
  
- define 0, 1 변경만 하면 같은 함수로 server, client 모두 동작함


  다른 칩으로 이식할 때 변경점

  1. w5500.h 에 #include "stm32f4xx_hal.h" mcu에 맞게 수정.  
  2. w5500.c 에 extern SPI_HandleTypeDef hspi1;  부분 프로젝트의 spi 설정에 맞게 수정하기. *spi 속도가 12MHz 이상이 되면 동작에 문제가 생길 수 있음.
  3. 끝!  

