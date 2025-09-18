// ==================== w5500_driver.h ====================
//bluefish parkjinhyeok 2025_09_16

#ifndef __W5500_DRIVER_H__
#define __W5500_DRIVER_H__


#include "main.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>


// ====== TCP 설정 ======
#define SERVER 0 // 1 = 서버, 0 = 클라이언트

#define SOCK_TCPS 0 // 소켓 번호 (0~7)


// ====== 소켓 레지스터 매크로 ======
#define Sn_MR(sn) (0x0000 + (sn<<5))
#define Sn_CR(sn) (0x0001 + (sn<<5))
#define Sn_SR(sn) (0x0003 + (sn<<5))
#define Sn_PORT(sn) (0x0004 + (sn<<5))
#define Sn_DIPR(sn) (0x000C + (sn<<5))
#define Sn_DPORT(sn) (0x0010 + (sn<<5))
#define Sn_TX_WR(sn) (0x0024 + (sn<<5))
#define Sn_RX_RSR(sn) (0x0026 + (sn<<5))
#define Sn_RX_RD(sn) (0x0028 + (sn<<5))


#define CMD_OPEN 0x01
#define CMD_LISTEN 0x02
#define CMD_CONNECT 0x04
#define CMD_DISCON 0x08
#define CMD_CLOSE 0x10
#define CMD_SEND 0x20
#define CMD_RECV 0x40


#define SOCK_CLOSED 0x00
#define SOCK_INIT 0x13
#define SOCK_LISTEN 0x14
#define SOCK_ESTABLISHED 0x17

#define IP_FRAME_LEN 18


void W5500_Process_IP_Frame(uint8_t *frame, uint16_t len);
void W5500_Reset();
void W5500_SetNetworkInfo();
void socket_open(uint8_t sn);
void socket_send(uint8_t sn, const uint8_t *buf, uint16_t len);
uint16_t socket_recv(uint8_t sn, uint8_t *buf, uint16_t bufsize);


#endif // __W5500_DRIVER_H__

