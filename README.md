# SmartWindow

STM32F446RE 기반 스마트 창문 제어 시스템.

온습도, 조도, 미세먼지, 빗물 상태를 주기적으로 읽고, 설정한 조건에 따라 창문과 블라인드를 제어한다.

수동 제어는 IR 리모컨으로 처리하고, 외부 장치와의 통신은 LoRa를 사용한다.

---

## Hardware

- MCU: STM32F446RE
- Temperature / Humidity: DHT11
- Light Sensor
- Dust Sensor
- Rain Sensor
- IR Receiver
- LoRa Module
- Window Motor
- Blind Motor

---

## Firmware

- C
- STM32 HAL
- STM32CubeMX / STM32CubeIDE

사용 peripheral:

- GPIO
- ADC
- UART
- SPI
- RTC
- TIM Input Capture
- General Purpose Timer

---

## System Overview

```text
                 +-------------------+
                 |   Environment     |
                 |      Sensors      |
                 +---------+---------+
                           |
                           v
                  +--------+--------+
                  |   STM32F446RE   |
                  |                 |
        IR ------>|                 |------> Window Motor
                  |                 |
      LoRa <----->|                 |------> Blind Motor
                  +--------+--------+
                           |
                           v
                     Debug UART
```

펌웨어의 주요 역할은 다음과 같다.

1. 센서 데이터 수집
2. 자동 제어 조건 판단
3. 창문 / 블라인드 구동
4. IR 명령 처리
5. LoRa 송수신
6. UART 로그 출력

---

## Main Loop

메인 루프에서는 모든 작업을 매 반복마다 수행하지 않는다.

센서 측정과 자동 제어는 `HAL_GetTick()`을 기준으로 일정 주기마다 실행한다.

```c
current_tick = HAL_GetTick();

if (current_tick - last_sensor_tick >= SENSOR_CHECK_INTERVAL)
{
    last_sensor_tick = current_tick;
    Handle_Periodic_Sensor_Task();
}
```

이 방식으로 센서 측정 주기와 메인 루프 실행 주기를 분리했다.

IR 입력은 interrupt 기반으로 수집한 결과를 메인 루프에서 처리한다.

```text
while(1)

  periodic sensor task
        |
        v
  sensor measurement
        |
        v
  auto control

  IR command check
        |
        v
  manual control
```

---

## Window / Blind Control

창문과 블라인드는 각각 방향 제어 GPIO와 동작 GPIO를 사용한다.

### Window

```text
OPEN
CLOSE
STOP
```

### Blind

```text
UP
DOWN
STOP
```

자동 모드가 활성화된 상태에서는 IR 수동 명령을 무시하도록 구성했다.

이를 통해 자동 제어와 수동 제어가 동시에 actuator를 건드리는 상황을 막았다.

---

## Sensor Processing

자동 모드에서는 다음 센서 값을 기반으로 제어한다.

```text
Temperature
Humidity
Dust
Rain
Light
```

센서별 threshold는 firmware 상수로 관리한다.

```c
#define TEMP_HOT_TO_OPEN    28
#define TEMP_COLD_TO_CLOSE  22
#define HUMI_TO_OPEN        40
#define HUMI_TO_CLOSE       25
#define LIGHT_BRIGHT_TH     4000
#define LIGHT_DARK_TH       1000
#define DUST_BAD_LEVEL      40
```

현재 구현은 rule-based control이다.

환경 조건을 단순히 하나씩 보는 것이 아니라, 여러 입력값을 조합해 창문과 블라인드 상태를 결정하도록 구성했다.

---

## IR Decode

IR 수신에는 TIM2 Input Capture를 사용한다.

pulse edge가 발생할 때 timer 값을 capture하고, capture 간 시간 차이를 이용해 수신 데이터를 복원한다.

관련 상태는 다음과 같이 관리한다.

```c
typedef enum {
    IR_STATE_IDLE,
    IR_STATE_AGC_SPACE,
    IR_STATE_DATA,
    IR_STATE_REPEAT_PULSE
} IR_DecoderState_t;
```

interrupt context와 main loop 사이에서 공유하는 값은 `volatile`로 선언했다.

```c
static volatile IR_DecoderState_t ir_state;
static volatile uint8_t ir_bit_count;
static volatile uint32_t ir_data_raw;
volatile uint8_t ir_command;
```

ISR에서는 가능한 한 수신 상태와 데이터만 갱신하고, 실제 actuator 제어는 main context에서 수행하도록 구성했다.

---

## LoRa Communication

LoRa 모듈은 SPI를 통해 제어한다.

통신 상태는 아래와 같이 분리했다.

```c
typedef enum {
    LORA_IDLE,
    LORA_TX,
    LORA_RX,
    LORA_WAIT_ACK
} LoRaState_t;
```

송신이 끝나면 바로 수신 모드로 전환한다.

```text
IDLE
  |
  v
 TX
  |
  v
WAIT_ACK
  |
  +------ RX Done ------> IDLE
  |
  +------ Timeout ------> IDLE
```

센서 데이터는 문자열 형태로 전송한다.

```text
T:25 H:55 D:18 R:0 L:2310 WDIR:1 BDIR:0
```

수신 명령은 다음과 같이 처리한다.

```text
OPEN
CLOSE
UP
DOWN
```

LoRa 명령이 들어오면 자동 모드를 해제한 뒤 해당 actuator를 직접 제어한다.

---

## Peripheral Mapping

| Peripheral | Usage |
|---|---|
| GPIO | Window / Blind motor control |
| ADC1 | Light sensor |
| UART2 | Debug log |
| SPI1 | LoRa |
| TIM2 | IR Input Capture |
| TIM3 | LoRa timing |
| TIM4 | Dust sensor pulse measurement |
| TIM6 | microsecond delay |
| RTC | time-related control |

---

## Source Layout

```text
Core/
├── Inc/
└── Src/
    ├── main.c
    ├── lora_app.c
    ├── stm32f4xx_it.c
    └── stm32f4xx_hal_msp.c
```

### `main.c`

- peripheral initialization
- sensor processing
- automatic control
- IR command handling
- window / blind control

### `lora_app.c`

- LoRa initialization
- TX / RX callback
- communication state management
- sensor data transmission
- remote command processing

---

## Design Notes

### 1. ISR와 actuator 제어 분리

interrupt 안에서 motor 제어까지 수행하지 않고, ISR에서는 입력 데이터 처리에 필요한 상태만 갱신한다.

실제 제어는 main context에서 수행한다.

### 2. 상태 기반 통신 처리

LoRa 송수신을 단순한 함수 호출 순서로 처리하지 않고 상태값으로 관리했다.

전송 중 재전송이나 수신 대기 중 중복 동작을 줄이기 위한 구조다.

### 3. 센서 처리 주기 분리

센서 읽기를 매 loop마다 수행하지 않고 `HAL_GetTick()` 기반 주기로 분리했다.

센서별 응답 속도와 시스템 responsiveness를 고려하기 위한 선택이다.

---

## Known Limitations

현재 구현은 **bare-metal super loop 구조**다.

따라서 기능이 추가되면 main loop가 복잡해지고, 통신 / 센서 / 제어 사이의 실행 시간 간섭이 커질 수 있다.

또 일부 sensor driver 및 delay 로직은 blocking 방식이다.

실제 제품 수준으로 확장한다면 다음 부분을 먼저 개선할 것이다.

1. FreeRTOS 적용
2. Sensor / Control / Communication task 분리
3. Queue 기반 데이터 전달
4. Mutex 또는 Semaphore를 통한 shared resource 보호
5. Motor control state machine 구성
6. LoRa packet binary format 적용
7. CRC / sequence number 추가
8. timeout / retry 정책 명확화
9. hardware-dependent code와 control logic 분리
10. unit-test 가능한 구조로 refactoring

---

## What I Learned

이 프로젝트를 통해 STM32 peripheral을 각각 사용하는 것보다, 여러 입력과 출력이 동시에 동작하는 시스템에서 **실행 흐름과 상태를 관리하는 것이 더 중요하다는 점**을 배웠다.

특히 아래 부분을 실제 코드로 다뤘다.

- polling과 interrupt의 역할 분리
- timer 기반 pulse measurement
- interrupt context와 main context 간 데이터 공유
- SPI 기반 wireless module 제어
- actuator와 sensor logic의 결합
- 통신 state 관리
- blocking code가 시스템 응답성에 미치는 영향
