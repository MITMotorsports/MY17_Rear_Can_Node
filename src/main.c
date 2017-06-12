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

// 48 MHz (for 32 bit interrupts) = 48M cycles per second
#define CYCLES_PER_MICROSECOND 48
// Microsecond = 1 millionth of a second
#define MICROSECONDS_PER_SECOND_F 1000000.0
// 1000 millirevs = 1 rev
#define MILLIREVS_PER_REV_F 1000.0
// TODO count number of teeth on wheel
#define CLICKS_PER_REV 23
// Pointless comment to not break pattern
#define SECONDS_PER_MINUTE 60

volatile uint32_t msTicks;

// integers in [0:4294967296] representing the number of clock cycles between
// ticks from wheel speed sensors
volatile uint32_t wheel_1_clock_cycles_between_ticks = 0;
volatile uint32_t wheel_2_clock_cycles_between_ticks = 0;

uint32_t last_heartbeat_ms = 0;
uint32_t last_wheel_speed_ms = 0;

static bool resettingPeripheral = false;

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
  wheel_1_clock_cycles_between_ticks = Chip_TIMER_ReadCapture(LPC_TIMER32_0, 0);
  Serial_Println("Wheel 1");
}

// Interrupt handler for timer 1 capture pin. This function get called automatically on
// a rising edge of the signal going into the timer capture pin
void TIMER32_1_IRQHandler(void) {
  Chip_TIMER_Reset(LPC_TIMER32_1);            /* Reset the timer immediately */
  Chip_TIMER_ClearCapture(LPC_TIMER32_1, 0);      /* Clear the capture */
  wheel_2_clock_cycles_between_ticks = Chip_TIMER_ReadCapture(LPC_TIMER32_1, 0);
  Serial_Println("Wheel 2");
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

void handle_can_error(Can_ErrorID_T error) {
  if (error != Can_Error_NONE && error != Can_Error_NO_RX) {
    /* Serial_Print("can_write_err: "); */
    /* Serial_PrintlnNumber(error, 16); */
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
    const uint32_t left_click_time = wheel_1_clock_cycles_between_ticks;
    const uint32_t right_click_time = wheel_2_clock_cycles_between_ticks;

    Can_RearCanNode_WheelSpeed_T msg;

    msg.rear_left_wheel_speed_mRPM = click_time_to_mRPM(left_click_time);
    msg.rear_right_wheel_speed_mRPM = click_time_to_mRPM(right_click_time);

    return Can_RearCanNode_WheelSpeed_Write(&msg);
}

Can_ErrorID_T write_can_heartbeat(void) {
    Can_RearCanNode_Heartbeat_T msg;
    msg.is_alive = true;
    return Can_RearCanNode_Heartbeat_Write(&msg);
}

uint32_t click_time_to_mRPM(uint32_t cycles_per_click) {
  if (cycles_per_click == 0) {
    return 0;
  }

  const uint32_t us_per_click = cycles_per_click / CYCLES_PER_MICROSECOND;
  const uint32_t us_per_rev = us_per_click * CLICKS_PER_REV;

  const float s_per_rev = us_per_rev / MICROSECONDS_PER_SECOND_F;

  const uint32_t mrev_per_s = (uint32_t)
    (MILLIREVS_PER_REV_F / s_per_rev);

  const uint32_t mrev_per_min = mrev_per_s * SECONDS_PER_MINUTE;
  return mrev_per_min;
}

