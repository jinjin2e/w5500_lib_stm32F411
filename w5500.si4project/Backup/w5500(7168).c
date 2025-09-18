// ==================== w5500_driver.c ====================
#include "w5500.h"

extern SPI_HandleTypeDef hspi1;


typedef struct {
    uint8_t local_ip[4];
    uint8_t subnet_mask[4];
    uint8_t gateway[4];
    uint16_t port;
} net_config_t;

net_config_t sys_config;

extern UART_HandleTypeDef huart1;  // ← 이 줄 추가



void uart_send_bytes(uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit(&huart1, data, len, 100);
}

void W5500_Select()   { HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_RESET); }
void W5500_Unselect() { HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_SET);   }

void W5500_Reset() {
    HAL_GPIO_WritePin(W5500_RST_GPIO_Port, W5500_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(W5500_RST_GPIO_Port, W5500_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(250);
}

uint8_t SPI_ReadWrite(uint8_t data) {
    uint8_t rx;
    HAL_SPI_TransmitReceive(&hspi1, &data, &rx, 1, HAL_MAX_DELAY);
    return rx;
}

void W5500_WriteBuf(uint16_t addr, uint8_t cb, const uint8_t *buf, uint16_t len) {
    W5500_Select();
    uint8_t header[3] = { addr >> 8, addr & 0xFF, cb };
    HAL_SPI_Transmit(&hspi1, header, 3, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi1, (uint8_t *)buf, len, HAL_MAX_DELAY);
    W5500_Unselect();
}

uint8_t W5500_ReadReg(uint16_t addr, uint8_t cb) {
    W5500_Select();
    uint8_t header[3] = { addr >> 8, addr & 0xFF, cb };
    HAL_SPI_Transmit(&hspi1, header, 3, HAL_MAX_DELAY);
    uint8_t data = SPI_ReadWrite(0xFF);
    W5500_Unselect();
    return data;
}

void W5500_ReadBuf(uint16_t addr, uint8_t cb, uint8_t *buf, uint16_t len) {
    W5500_Select();
    uint8_t header[3] = { addr >> 8, addr & 0xFF, cb };
    HAL_SPI_Transmit(&hspi1, header, 3, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, buf, len, HAL_MAX_DELAY);
    W5500_Unselect();
}

void W5500_WriteReg(uint16_t addr, uint8_t cb, uint8_t data) {
    W5500_Select();
    uint8_t header[3] = { addr >> 8, addr & 0xFF, cb };
    HAL_SPI_Transmit(&hspi1, header, 3, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi1, &data, 1, HAL_MAX_DELAY);
    W5500_Unselect();
}

void W5500_SetNetworkInfo() {
    const uint8_t MAC[6] = {0x00, 0x08, 0xDC, 0x01, 0x02, 0x03};
    const uint8_t IP[4]  = {192, 168, 0, 199};
    const uint8_t GW[4]  = {192, 168, 0, 1};
    const uint8_t SN[4]  = {255, 255, 255, 0};

    W5500_WriteBuf(0x0009, 0x04, MAC, 6);
    W5500_WriteBuf(0x000F, 0x04, IP, 4);
    W5500_WriteBuf(0x0001, 0x04, GW, 4);
    W5500_WriteBuf(0x0005, 0x04, SN, 4);
}

void socket_open(uint8_t sn) {
#if SERVER
    W5500_WriteReg(Sn_MR(sn), 0x0C, 0x01);
    uint8_t port[2] = { TCP_PORT >> 8, TCP_PORT & 0xFF };
    W5500_WriteBuf(Sn_PORT(sn), 0x0C, port, 2);
    W5500_WriteReg(Sn_CR(sn), 0x0C, CMD_OPEN);
    HAL_Delay(5);
    W5500_WriteReg(Sn_CR(sn), 0x0C, CMD_LISTEN);
    printf("TCP Server opened on port %d\r\n", TCP_PORT);
#else
    W5500_WriteReg(Sn_MR(sn), 0x0C, 0x01);
    uint8_t port[2] = { TCP_PORT >> 8, TCP_PORT & 0xFF };
    W5500_WriteBuf(Sn_PORT(sn), 0x0C, port, 2);

    uint8_t dip[4] = {DEST_IP0, DEST_IP1, DEST_IP2, DEST_IP3};
    W5500_WriteBuf(Sn_DIPR(sn), 0x0C, dip, 4);
    uint8_t dport[2] = { TCP_PORT >> 8, TCP_PORT & 0xFF };
    W5500_WriteBuf(Sn_DPORT(sn), 0x0C, dport, 2);

    W5500_WriteReg(Sn_CR(sn), 0x0C, CMD_OPEN);
    HAL_Delay(5);
    W5500_WriteReg(Sn_CR(sn), 0x0C, CMD_CONNECT);
    printf("TCP Client connecting to %d.%d.%d.%d:%d\r\n", DEST_IP0, DEST_IP1, DEST_IP2, DEST_IP3, TCP_PORT);
#endif
}

void socket_send(uint8_t sn, const uint8_t *buf, uint16_t len) {
    uint8_t ptr[2];
    W5500_ReadBuf(Sn_TX_WR(sn), 0x08, ptr, 2);
    uint16_t offset = (ptr[0]<<8) | ptr[1];
    uint16_t tx_addr = (sn * 0x0800) + (offset & 0x07FF);

    W5500_WriteBuf(tx_addr, 0x14, buf, len);

    offset += len;
    ptr[0] = offset >> 8;
    ptr[1] = offset & 0xFF;
    W5500_WriteBuf(Sn_TX_WR(sn), 0x0C, ptr, 2);

    W5500_WriteReg(Sn_CR(sn), 0x0C, CMD_SEND);
    while (W5500_ReadReg(Sn_CR(sn), 0x0C));
}

uint16_t socket_recv(uint8_t sn, uint8_t *buf, uint16_t bufsize) {
    uint8_t rsr[2];
    W5500_ReadBuf(Sn_RX_RSR(sn), 0x08, rsr, 2);
    uint16_t recv_size = (rsr[0]<<8) | rsr[1];
    if (recv_size == 0) return 0;
    if (recv_size > bufsize) recv_size = bufsize;

    uint8_t rd[2];
    W5500_ReadBuf(Sn_RX_RD(sn), 0x08, rd, 2);
    uint16_t offset = (rd[0]<<8) | rd[1];

    uint16_t rx_addr = (sn<<10) + (offset & 0x7FF);
    W5500_ReadBuf(rx_addr, 0x18, buf, recv_size);

    offset += recv_size;
    rd[0] = offset >> 8;
    rd[1] = offset & 0xFF;
    W5500_WriteBuf(Sn_RX_RD(sn), 0x0C, rd, 2);

    W5500_WriteReg(Sn_CR(sn), 0x0C, CMD_RECV);
    while (W5500_ReadReg(Sn_CR(sn), 0x0C));

    return recv_size;
}



// STX/ETX/명령어 정의
#define STX       0x53
#define ETX       0x45
#define CMD_SET   0x10
#define CMD_GET   0x20

// 유틸: checksum 계산
static uint8_t calc_checksum(const uint8_t *data, uint8_t len) {
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; ++i) {
        sum += data[i];
    }
    return sum;
}

// 수신된 IP 세팅 프레임 처리
void W5500_Process_IP_Frame(uint8_t *frame, uint16_t len) {
    if (len != IP_FRAME_LEN) return;
    if (frame[0] != STX || frame[17] != ETX) return;

    uint8_t checksum = calc_checksum(frame, 16);
    if (checksum != frame[16]) return;

    uint8_t cmd = frame[1];

    if (cmd == CMD_SET) {
        uint8_t ip[4]   = { frame[2],  frame[3],  frame[4],  frame[5]  };
        uint8_t mask[4] = { frame[6],  frame[7],  frame[8],  frame[9]  };
        uint8_t gw[4]   = { frame[10], frame[11], frame[12], frame[13] };
        uint16_t port   = (frame[14] << 8) | frame[15];

        // W5500에 설정 반영
        setSHAR((uint8_t[]){0x00, 0x08, 0xDC, 0x00, 0x00, 0x01}); // MAC (고정)
        setGAR(gw);
        setSUBR(mask);
        setSIPR(ip);

        // 내부 저장
        memcpy(sys_config.local_ip, ip, 4);
        memcpy(sys_config.subnet_mask, mask, 4);
        memcpy(sys_config.gateway, gw, 4);
        sys_config.port = port;

        // ACK 프레임 전송
        uint8_t tx[IP_FRAME_LEN] = {0};
        tx[0] = STX;
        tx[1] = 0x11; // SET+1
        memcpy(&tx[2],  ip,   4);
        memcpy(&tx[6],  mask, 4);
        memcpy(&tx[10], gw,   4);
        tx[14] = (uint8_t)((port >> 8) & 0xFF);
        tx[15] = (uint8_t)(port & 0xFF);
        tx[16] = calc_checksum(tx, 16);
        tx[17] = ETX;

        uart_send_bytes(tx, IP_FRAME_LEN);
    }

    else if (cmd == CMD_GET) {
        uint8_t ip[4], mask[4], gw[4];
        getSIPR(ip);
        getSUBR(mask);
        getGAR(gw);
        uint16_t port = sys_config.port;

        uint8_t tx[IP_FRAME_LEN] = {0};
        tx[0] = STX;
        tx[1] = 0x21; // GET+1
        memcpy(&tx[2],  ip,   4);
        memcpy(&tx[6],  mask, 4);
        memcpy(&tx[10], gw,   4);
        tx[14] = (uint8_t)((port >> 8) & 0xFF);
        tx[15] = (uint8_t)(port & 0xFF);
        tx[16] = calc_checksum(tx, 16);
        tx[17] = ETX;

        uart_send_bytes(tx, IP_FRAME_LEN);
    }
}

