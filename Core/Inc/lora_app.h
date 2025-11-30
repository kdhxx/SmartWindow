/* lora_app.h */
#ifndef LORA_APP_H
#define LORA_APP_H

#include "main.h" // HAL 드라이버 및 핀 정의 사용

#define RF_FREQUENCY                                922100000 // Hz
#define TX_OUTPUT_POWER                             14        // dBm
#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false
#define RX_TIMEOUT_VALUE                            10000
#define BUFFER_SIZE                                 64		// Define the payload size here

#define Rx_ID			77		// 공유 아이디
#define PHYMAC_PDUOFFSET_RXID          				0		// packet 내부 아이디 offset

// 메인에서 갖다 쓸 함수들
void LoRa_Init_User(void);        // LoRa 초기화
void LoRa_Process_Task(void);     // LoRa 상태머신 (Loop 안에서 계속 돌릴 것)
void LoRa_Send_SensorData(int t_int, int h_int, int d_int, int rain, int light, int w, int b);

#endif
