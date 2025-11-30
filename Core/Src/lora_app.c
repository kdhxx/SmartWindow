/* lora_app.c */
#include "lora_app.h"
#include "sx1272/radio.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "main.h"

static uint8_t Buffer[BUFFER_SIZE];
static RadioEvents_t RadioEvents;

int8_t RssiValue = 0;
int8_t SnrValue = 0;

// 상태 관리용
typedef enum {
    LORA_IDLE, // 0: 쉴 때
    LORA_TX, // 1: 보낼 때
    LORA_RX, // 2: 받을 때
    LORA_WAIT_ACK // 3: 답장 기다릴 때
} LoRaState_t;

static LoRaState_t State = LORA_IDLE;

// 외부 모터 함수 연결
extern void Window_Action_Open(void);
extern void Window_Action_Close(void);
extern void Blind_Action_Up(void);
extern void Blind_Action_Down(void);
extern int auto_mode;

// --- [콜백 함수들] ---
void OnTxDone(void) {
    Radio.Sleep();

    printf("> [LoRa] Tx Done! Listen...\r\n");
    Radio.Rx(RX_TIMEOUT_VALUE); // 전송 후 즉시 수신 대기
    State = LORA_WAIT_ACK;
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
    Radio.Sleep();
    char rxCmd[64];
    memset(rxCmd, 0, sizeof(rxCmd));
    if(payload[0] == Rx_ID) {
		if (size > 1)
		{
			memcpy(rxCmd, payload + 1, size - 1); // ID 떼고 복사
		}

		printf("> [LoRa] RX Cmd: %s\r\n", rxCmd);

		if (strncmp(rxCmd, "OPEN", 4) == 0)
		{
			auto_mode = 0;
			Window_Action_Open();
		}
		else if(strncmp(rxCmd, "CLOSE", 5) == 0)
		{
			auto_mode = 0;
			Window_Action_Close();
		}
		else if(strncmp(rxCmd, "UP", 2) == 0)
		{
			auto_mode = 0;
			Blind_Action_Up();
		}
		else if(strncmp(rxCmd, "DOWN", 4) == 0)
		{
			auto_mode = 0;
			Blind_Action_Down();
		}
		State = LORA_IDLE; // 다시 대기 상태로
    }
}

void OnTxTimeout(void) { Radio.Sleep(); State = LORA_IDLE; }
void OnRxTimeout(void) { Radio.Sleep(); State = LORA_IDLE; printf("> [LoRa] Rx Timeout\r\n"); }
void OnRxError(void)   { Radio.Sleep(); State = LORA_IDLE; }

// 1. 초기화 함수
void LoRa_Init_User(void) {
    // 라디오 콜백 연결
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;

    Radio.Init(&RadioEvents);
    Radio.SetChannel( RF_FREQUENCY ); // 주파수

    // TX/RX 설정
    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
    									 LORA_SPREADING_FACTOR, LORA_CODINGRATE,
    									 LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
    									 true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
    									 LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
    									 LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
    									 0, true, 0, 0, LORA_IQ_INVERSION_ON, true );

    Buffer[PHYMAC_PDUOFFSET_RXID] = Rx_ID;
    printf("set ID : %d\n", Buffer[0]);

    printf("=== LoRa Initialized ===\r\n");
}

// 2. 데이터 전송 요청 함수 (인자 7개 처리)
void LoRa_Send_SensorData(int t_int, int h_int, int d_int, int rain, int light, int w, int b) {
    // 보낼 데이터가 있을 때만 TX 상태로 전환
    if(State == LORA_IDLE) {

        memset((void *)(Buffer + 1), 0, BUFFER_SIZE -1);
        sprintf((char*)Buffer + 1, "T:%d H:%d D:%d R:%d L:%d WDIR:%d BDIR:%d\r\n",
                t_int,
                h_int,
                d_int,
                rain,
				light,
				w,
				b);

        printf("> [LoRa] Sending: %s\r\n", (char*)(Buffer+1));
    	Radio.Send(Buffer, BUFFER_SIZE);
        State = LORA_TX;
    }
    else {
        // 전송 중이거나 수신 대기 중일 때는 무시 (충돌 방지)
        printf("[LoRa] Busy! State: %d\r\n", State);
        State = LORA_IDLE;
    }
}
