# w5500_lib_stm32F411
w5500 간단한 라이브러리  
  
- define 0, 1 변경만 하면 같은 함수로 server, client 모두 동작함


  다른 칩으로 이식할 때 변경점

  1. w5500.h 에 #include "stm32f4xx_hal.h" mcu에 맞게 수정.  
  2. w5500.c 에 extern SPI_HandleTypeDef hspi1;  부분 프로젝트의 spi 설정에 맞게 수정하기. *spi 속도가 12MHz 이상이 되면 동작에 문제가 생길 수 있음.
  3. spi 포트와, cs, reset 등 라이브러리에서 사용하고 있는 포트 설정해주기.
  4. 끝!  

--------------

* 만약 flash나 다른 저장장치를 사용해서 전원이 꺼져도 유지되는 동작을 추가하려면

```
  void socket_open(uint8_t sn) {
#if SERVER
    // === ���� ��� ===
    W5500_WriteReg(Sn_MR(sn), 0x0C, 0x01);  // TCP
    uint8_t port[2] = { sys_config.port >> 8, sys_config.port & 0xFF };
    W5500_WriteBuf(Sn_PORT(sn), 0x0C, port, 2);
    W5500_WriteReg(Sn_CR(sn), 0x0C, CMD_OPEN);
    HAL_Delay(5);
    W5500_WriteReg(Sn_CR(sn), 0x0C, CMD_LISTEN);
    printf("TCP Server opened on port %d\r\n", sys_config.port);
#else
    // === Ŭ���̾�Ʈ ��� ===
    W5500_WriteReg(Sn_MR(sn), 0x0C, 0x01);  // TCP

    // Ŭ���̾�Ʈ ���ε� ��Ʈ �� W5500�� �ڵ����� �����ص� ������ ���������� �Ҵ� ����
    uint8_t port[2] = { 0x00, 0x00 };
    W5500_WriteBuf(Sn_PORT(sn), 0x0C, port, 2);

    // ������ IP / ��Ʈ
    W5500_WriteBuf(Sn_DIPR(sn), 0x0C, client_config.dest_ip, 4);
    uint8_t dport[2] = { client_config.dest_port >> 8, client_config.dest_port & 0xFF };
    W5500_WriteBuf(Sn_DPORT(sn), 0x0C, dport, 2);

    W5500_WriteReg(Sn_CR(sn), 0x0C, CMD_OPEN);
    HAL_Delay(5);
    W5500_WriteReg(Sn_CR(sn), 0x0C, CMD_CONNECT);
    printf("TCP Client connecting to %d.%d.%d.%d:%d\r\n",
           client_config.dest_ip[0], client_config.dest_ip[1],
           client_config.dest_ip[2], client_config.dest_ip[3],
           client_config.dest_port);
#endif
}
```
여기서 저장을 하고, 

```
void sys_config_init() { // here add flash  -  save and load value
    sys_config.local_ip[0] = 192;
    sys_config.local_ip[1] = 168;
    sys_config.local_ip[2] = 0;
    sys_config.local_ip[3] = 199;

    sys_config.subnet_mask[0] = 255;
    sys_config.subnet_mask[1] = 255;
    sys_config.subnet_mask[2] = 255;
    sys_config.subnet_mask[3] = 0;

    sys_config.gateway[0] = 192;
    sys_config.gateway[1] = 168;
    sys_config.gateway[2] = 0;
    sys_config.gateway[3] = 1;

    sys_config.port = 1010;
	
    client_config.dest_ip[0] = 192;
    client_config.dest_ip[1] = 168;
    client_config.dest_ip[2] = 0;
    client_config.dest_ip[3] = 23;

    client_config.dest_port = 1010;
}
```
해당 함수에서 저장한 값을 불러오면 된다.   
