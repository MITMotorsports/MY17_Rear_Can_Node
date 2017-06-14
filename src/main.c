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

#define WHEEL_SPEED_TIMEOUT_MS 100

// 48 MHz (for 32 bit interrupts) = 48M cycles per second
#define CYCLES_PER_MICROSECOND 48
// Microsecond = 1 millionth of a second
#define MICROSECONDS_PER_SECOND_F 1000000.0
// 1000 millirevs = 1 rev
#define MILLIREVS_PER_REV_F 1000.0
// TODO count number of teeth on wheel
#define CLICKS_PER_REV 27
// Pointless comment to not break pattern
#define SECONDS_PER_MINUTE 60

volatile uint32_t msTicks;

// integers in [0:4294967296] representing the number of clock cycles between
// ticks from wheel speed sensors
volatile uint32_t wheel_1_clock_cycles_between_ticks = 0;
volatile uint32_t wheel_2_clock_cycles_between_ticks = 0;

uint32_t last_heartbeat_ms = 0;
uint32_t last_wheel_speed_ms = 0;

volatile uint32_t last_wheel_1_click = 0;
volatile uint32_t last_wheel_2_click = 0;

// We want to ensure the first click after a timeout is disregarded.
// This prevents potential overflow of the timer causing a falsely-low
// gap between clicks, which would give a falsely-high RPM.
volatile bool wheel_1_disregard = false;
volatile bool wheel_2_disregard = false;

static bool resettingPeripheral = false;

void can_read(void);
void handle_can_error(Can_ErrorID_T error);
Can_ErrorID_T write_can_heartbeat(void);
Can_ErrorID_T write_can_wheel_speed(void);
uint32_t click_time_to_mRPM(uint32_t click_time);

/*****************************************************************************/

 /* Private function */
void SysTick_Handler(void) {
  msTicks++;
}

/****************************************************************************/


// Interrupt handler for timer 0 capture pin. This function get called automatically on
// a rising edge of the signal going into the timer capture pin
void TIMER32_0_IRQHandler(void) {
  Chip_TIMER_Reset(LPC_TIMER32_0);            /* Reset the timer immediately */
  Chip_TIMER_ClearCapture(LPC_TIMER32_0, 0);      /* Clear the capture */
  if (wheel_1_disregard) {
    wheel_1_clock_cycles_between_ticks = 0;
  } else {
    wheel_1_clock_cycles_between_ticks = Chip_TIMER_ReadCapture(LPC_TIMER32_0, 0);
  }
  last_wheel_1_click = msTicks;
}

// Interrupt handler for timer 1 capture pin. This function get called automatically on
// a rising edge of the signal going into the timer capture pin
void TIMER32_1_IRQHandler(void) {
  Chip_TIMER_Reset(LPC_TIMER32_1);            /* Reset the timer immediately */
  Chip_TIMER_ClearCapture(LPC_TIMER32_1, 0);      /* Clear the capture */
  if (wheel_2_disregard) {
    wheel_2_clock_cycles_between_ticks = 0;
  } else {
    wheel_2_clock_cycles_between_ticks = Chip_TIMER_ReadCapture(LPC_TIMER32_1, 0);
  }
  last_wheel_2_click = msTicks;
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

    // Capture values
    const uint32_t wheel_1_click_time = wheel_1_clock_cycles_between_ticks;
    const uint32_t wheel_1_last_updated = last_wheel_1_click;
    const uint32_t wheel_2_click_time = wheel_2_clock_cycles_between_ticks;
    const uint32_t wheel_2_last_updated = last_wheel_2_click;

    const bool wheel_1_timeout =
      wheel_1_last_updated + WHEEL_SPEED_TIMEOUT_MS < msTicks;
    const bool wheel_2_timeout =
      wheel_2_last_updated + WHEEL_SPEED_TIMEOUT_MS < msTicks;

    if (wheel_1_timeout || wheel_1_click_time == 0) {
      msg.rear_right_wheel_speed_mRPM = 0;
    } else {
      msg.rear_right_wheel_speed_mRPM = click_time_to_mRPM(wheel_1_click_time);
    }
    wheel_1_disregard = wheel_1_timeout;

    if (wheel_2_timeout || wheel_2_click_time == 0) {
      msg.rear_left_wheel_speed_mRPM = 0;
    } else {
      msg.rear_left_wheel_speed_mRPM = click_time_to_mRPM(wheel_2_click_time);
    }
    wheel_2_disregard = wheel_2_timeout;

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

uint32_t click_time_to_mRPM(uint32_t cycles_per_click) {
  const float us_per_click = cycles_per_click * 1.0 / CYCLES_PER_MICROSECOND;
  const float us_per_rev = us_per_click * CLICKS_PER_REV;

  const float s_per_rev = us_per_rev / MICROSECONDS_PER_SECOND_F;

  const float mrev_per_s = MILLIREVS_PER_REV_F / s_per_rev;

  const float mrev_per_min = mrev_per_s * SECONDS_PER_MINUTE;
  return (uint32_t)(mrev_per_min);
}

