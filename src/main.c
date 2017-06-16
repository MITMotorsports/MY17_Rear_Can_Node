#include "chip.h"
#include "can.h"

#include "serial.h"
#include "timer.h"

#include <MY17_Can_Library.h>

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

const uint32_t OscRateIn = 12000000;

#define SERIAL_BAUDRATE 115200
#define CAN_BAUDRATE 500000

#define HEARTBEAT_PERIOD_MS 1000
#define WHEEL_SPEED_PERIOD_MS 20

typedef enum {
  RIGHT,
  LEFT,
  NUM_WHEELS
} Wheel_T;

#define NUM_TEETH 27
#define SUM_ALL_TEETH (NUM_TEETH * (NUM_TEETH + 1) / 2)
#define CYCLES_PER_MICROSECOND 48

#define WHEEL_SPEED_TIMEOUT_MS 100

// Microsecond = 1 millionth of a second
#define MICROSECONDS_PER_SECOND_F 1000000.0
// 1000 millirevs = 1 rev
#define MILLIREVS_PER_REV_F 1000.0
// Pointless comment to not break pattern
#define SECONDS_PER_MINUTE 60

volatile uint32_t msTicks;

// integers in [0:4294967296] representing the number of clock cycles between
// ticks from wheel speed sensors
volatile uint32_t last_tick[NUM_WHEELS][NUM_TEETH];

volatile uint32_t num_ticks[NUM_WHEELS];
volatile uint64_t big_sum[NUM_WHEELS];
volatile uint64_t little_sum[NUM_WHEELS];

volatile bool disregard[NUM_WHEELS];

volatile uint32_t last_updated[NUM_WHEELS];

uint32_t last_wheel_speed_ms = 0;
uint32_t last_heartbeat_ms = 0;

static bool resettingPeripheral = false;

void can_read(void);
void handle_can_error(Can_ErrorID_T error);
Can_ErrorID_T write_can_heartbeat(void);
Can_ErrorID_T write_can_wheel_speed(void);
uint32_t click_time_to_mRPM(uint32_t click_time_us);

/*****************************************************************************/

 /* Private function */
void SysTick_Handler(void) {
  msTicks++;
}

/****************************************************************************/

void handle_interrupt(LPC_TIMER_T* timer, Wheel_T wheel) {
  Chip_TIMER_Reset(timer);            /* Reset the timer immediately */
  Chip_TIMER_ClearCapture(timer, 0);      /* Clear the capture */
  const uint32_t curr_tick = Chip_TIMER_ReadCapture(timer, 0) / CYCLES_PER_MICROSECOND;

  // Interrupt can now proceed

  if (disregard[wheel]) {
    num_ticks[wheel] = 0;
    big_sum[wheel] = 0;
    little_sum[wheel] = 0;
    last_updated[wheel] = msTicks;
    return;
  }

  const uint32_t count = num_ticks[wheel];
  const uint8_t idx = count % NUM_TEETH;
  const uint32_t this_tooth_last_rev =
    count < NUM_TEETH ? 0 : last_tick[wheel][idx];

  // Register tick
  last_tick[wheel][idx] = curr_tick;
  num_ticks[wheel]++;

  // Update big sum
  big_sum[wheel] += NUM_TEETH * curr_tick;
  big_sum[wheel] -= little_sum[wheel];

  // Update little sum
  little_sum[wheel] += curr_tick;
  little_sum[wheel] -= this_tooth_last_rev;

  // Update timestamp
  last_updated[wheel] = msTicks;
}


// Interrupt handler for timer 0 capture pin. This function get called automatically on
// a rising edge of the signal going into the timer capture pin
void TIMER32_0_IRQHandler(void) {
  handle_interrupt(LPC_TIMER32_0, RIGHT);
}

// Interrupt handler for timer 1 capture pin. This function get called automatically on
// a rising edge of the signal going into the timer capture pin
void TIMER32_1_IRQHandler(void) {
  handle_interrupt(LPC_TIMER32_1, LEFT);
}

void Set_Interrupt_Priorities(void) {
  /* Give 32 bit timer capture interrupts the highest priority */
  NVIC_SetPriority(TIMER_32_0_IRQn, 0);
  NVIC_SetPriority(TIMER_32_1_IRQn, 1);
  /* Give the SysTick function a lower priority */
  NVIC_SetPriority(SysTick_IRQn, 2);	
}

int main(void) {

  SystemCoreClockUpdate();

  if (SysTick_Config (SystemCoreClock / 1000)) {
    //Error
    while(1);
  }

  Serial_Init(SERIAL_BAUDRATE);
  Can_Init(CAN_BAUDRATE);

  Timer_Init();
  Set_Interrupt_Priorities();
  Timer_Start();

  Serial_Println("Started up");

  while (1) {
    can_read();
    if (last_wheel_speed_ms + WHEEL_SPEED_PERIOD_MS < msTicks) {
      last_wheel_speed_ms = msTicks;
      handle_can_error(write_can_wheel_speed());
    }
    if (last_heartbeat_ms + HEARTBEAT_PERIOD_MS < msTicks) {
      last_heartbeat_ms = msTicks;
      handle_can_error(write_can_heartbeat());
    }
  }
}

void can_read(void) {
  Can_MsgID_T msgID = Can_MsgType();
  switch(msgID) {
    case Can_Error_Msg:
      // Clear error if there is one for some reason
      Can_Error_Read();
      break;
    case Can_No_Msg:
    default:
      break;
  }
}

void handle_can_error(Can_ErrorID_T error) {
  if (error != Can_Error_NONE && error != Can_Error_NO_RX) {
    if (!resettingPeripheral) {
      resettingPeripheral = true;
      // TODO add this to CAN library
      CAN_ResetPeripheral();
      Can_Init(500000);
    }
  } else {
    resettingPeripheral = false;
  }
}

Can_ErrorID_T write_can_wheel_speed(void) {

  Can_RearCanNode_WheelSpeed_T msg;

  uint8_t wheel;
  for (wheel = 0; wheel < NUM_WHEELS; wheel++) {
    uint32_t *ptr;
    if (wheel == LEFT) {
      ptr = &msg.rear_left_wheel_speed_mRPM;
    } else if (wheel == RIGHT) {
      ptr = &msg.rear_right_wheel_speed_mRPM;
    }

    const uint32_t count = num_ticks[wheel];
    uint8_t idx;
    if (count > 0) {
      // If there are x ticks so far, the last tick is index (x - 1)
      idx = (count - 1) % NUM_TEETH;
    } else {
      idx = 0;
    }

    const uint32_t tick_time = last_tick[wheel][idx];

    uint32_t moving_avg_us = 0;
    if (count >= NUM_TEETH) {
      moving_avg_us = big_sum[wheel] / SUM_ALL_TEETH;
    }

    const bool timeout =
      last_updated[wheel] + WHEEL_SPEED_TIMEOUT_MS < msTicks;
    disregard[wheel] = timeout;

    bool wheel_stopped = timeout || count == 0;
    if (wheel_stopped) {
      *ptr = 0;
    } else if (count < NUM_TEETH) {
      *ptr = click_time_to_mRPM(tick_time);
    } else {
      *ptr = click_time_to_mRPM(moving_avg_us);
    }
  }

  Serial_Print("Left: ");
  Serial_PrintNumber(msg.rear_left_wheel_speed_mRPM, 10);
  Serial_Print(", Right: ");
  Serial_PrintlnNumber(msg.rear_right_wheel_speed_mRPM, 10);

  return Can_RearCanNode_WheelSpeed_Write(&msg);
}

Can_ErrorID_T write_can_heartbeat(void) {
    Can_RearCanNode_Heartbeat_T msg;
    msg.is_alive = true;
    return Can_RearCanNode_Heartbeat_Write(&msg);
}

uint32_t click_time_to_mRPM(uint32_t us_per_click) {
  const float us_per_rev = us_per_click * 1.0 * NUM_TEETH;

  const float s_per_rev = us_per_rev / MICROSECONDS_PER_SECOND_F;

  const float mrev_per_s = MILLIREVS_PER_REV_F / s_per_rev;

  const float mrev_per_min = mrev_per_s * SECONDS_PER_MINUTE;
  return (uint32_t)(mrev_per_min);
}

