/**
 * fake-mouse
 * Copyright (C) Jon Cross, 2020
*/

/** \file
 * Main source file for fake-mouse. This is based on the Mouse demo code that comes with
 * LUFA. Instead of using a hardware JoyStick to provide mouse movements it uses a timer
 * to update a state machine to make small movements every 20ish seconds.
 */

/* Includes: */
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <string.h>

#include "Descriptors.h"

#include <LUFA/Drivers/USB/USB.h>
#include <LUFA/Platform/Platform.h>

/* Macros for LEDs */
#define RX_LED       (1 << PB0)
#define TX_LED       (1 << PD5)
#define LED_CONFIG() {(DDRD |= TX_LED); (DDRB |= RX_LED);}
#define RX_LedOn()   (PORTB |= RX_LED)
#define RX_LedOff()  (PORTB &= ~RX_LED)
#define TX_LedOff()   (PORTD |= TX_LED)
#define TX_LedOn()  (PORTD &= ~TX_LED)

#define TX_LedToggle()	(PIND |= TX_LED)
#define RX_LedToggle()	(PINB |= RX_LED)

/* fake mouse states */
enum
  {
   MOVE_NONE = 0,
   MOVE_RIGHT,
   MOVE_DOWN,
   MOVE_LEFT,
   MOVE_UP,
   MOVE_MAX
  };

#define TARGET_SECONDS 30
#define MOVE_AMOUNT 10
typedef struct s_stateMachine
{
  uint8_t currentSeconds;
  uint8_t nextMove;
  uint8_t lastMove;
  uint8_t moving;
} stateMachine_t;

volatile stateMachine_t state =
  {
   .currentSeconds = 0,
   .nextMove = MOVE_NONE,
   .lastMove = MOVE_NONE,
   .moving = 0
  };


/* Function Prototypes: */
void SetupHardware(void);
void TimerInit(void);

void EVENT_USB_Device_Connect(void);
void EVENT_USB_Device_Disconnect(void);
void EVENT_USB_Device_ConfigurationChanged(void);
void EVENT_USB_Device_ControlRequest(void);
void EVENT_USB_Device_StartOfFrame(void);

bool CALLBACK_HID_Device_CreateHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
					 uint8_t* const ReportID,
					 const uint8_t ReportType,
					 void* ReportData,
					 uint16_t* const ReportSize);
void CALLBACK_HID_Device_ProcessHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
					  const uint8_t ReportID,
					  const uint8_t ReportType,
					  const void* ReportData,
					  const uint16_t ReportSize);

/** Buffer to hold the previously generated Mouse HID report, for comparison purposes inside the HID class driver. */
static uint8_t PrevMouseHIDReportBuffer[sizeof(USB_MouseReport_Data_t)];

/** LUFA HID Class driver interface configuration and state information. This structure is
 *  passed to all HID Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_HID_Device_t Mouse_HID_Interface =
  {
   .Config =
   {
    .InterfaceNumber        = INTERFACE_ID_Mouse,
    .ReportINEndpoint      =
    {
     .Address               = MOUSE_EPADDR,
     .Size                  = MOUSE_EPSIZE,
     .Banks                 = 1,
    },
    .PrevReportINBuffer     = PrevMouseHIDReportBuffer,
    .PrevReportINBufferSize = sizeof(PrevMouseHIDReportBuffer),
   },
  };


/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
  GlobalInterruptDisable();
  SetupHardware();
  GlobalInterruptEnable();
  for (;;)
    {
      HID_Device_USBTask(&Mouse_HID_Interface);
      USB_USBTask();
    }
  return 0;
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
#if (ARCH == ARCH_AVR8)
  /* Disable watchdog if enabled by bootloader/fuses */
  MCUSR &= ~(1 << WDRF);
  wdt_disable();

  /* Disable clock division */
  clock_prescale_set(clock_div_1);
#endif
  LED_CONFIG();
  RX_LedOff();
  TX_LedOff();
  USB_Init();
  TimerInit();
}

/** Configure the timer to give us 1 second compare triggers */
void TimerInit(void)
{
  // Use timer1 16 bit.
  // Divide system clock by 1024 (16MHz/1024 = 15625 = 1s)
  TCCR1A = 0;
  TCCR1B = 0x05;

  TCCR1C = 0x80; // force compare A

  //OCR1AH = ((15625 << 8) & 0xff);
  //OCR1AL = (15625 & 0xff);
  OCR1A = 15625;
  TIMSK1 = 0x02; //output compare A match interrupt enable

  TCNT1 = 0;
}

/** Interrupt Service Routine for TIMER) compare */
ISR(TIMER1_COMPA_vect)
{
  /* reset counter */
  TCNT1 = 0;
  /* do calculations */
  state.currentSeconds++;
  if (state.currentSeconds == TARGET_SECONDS)
    {
      TX_LedOn();
      /* time to do movement */
      state.nextMove = state.lastMove + 1;
      state.moving = MOVE_AMOUNT;
      if (state.nextMove == MOVE_MAX) {
	state.nextMove = MOVE_RIGHT;
      }
      /* reset current seconds */
      state.currentSeconds = 0;
    }
}

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
  RX_LedOn();
  TX_LedOff();
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
  RX_LedOff();
  TX_LedOff();
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
  bool ConfigSuccess = true;

  ConfigSuccess &= HID_Device_ConfigureEndpoints(&Mouse_HID_Interface);

  USB_Device_EnableSOFEvents();

}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
  HID_Device_ProcessControlRequest(&Mouse_HID_Interface);
}

/** Event handler for the USB device Start Of Frame event. */
void EVENT_USB_Device_StartOfFrame(void)
{
  HID_Device_MillisecondElapsed(&Mouse_HID_Interface);
}

/** HID class driver callback function for the creation of HID reports to the host.
 *
 *  \param[in]     HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in,out] ReportID    Report ID requested by the host if non-zero, otherwise callback should set to the generated report ID
 *  \param[in]     ReportType  Type of the report to create, either HID_REPORT_ITEM_In or HID_REPORT_ITEM_Feature
 *  \param[out]    ReportData  Pointer to a buffer where the created report should be stored
 *  \param[out]    ReportSize  Number of bytes written in the report (or zero if no report is to be sent)
 *
 *  \return Boolean \c true to force the sending of the report, \c false to let the library determine if it needs to be sent
 */
bool CALLBACK_HID_Device_CreateHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
                                         uint8_t* const ReportID,
                                         const uint8_t ReportType,
                                         void* ReportData,
                                         uint16_t* const ReportSize)
{
  USB_MouseReport_Data_t* MouseReport = (USB_MouseReport_Data_t*)ReportData;

  if (state.nextMove != MOVE_NONE)
    {
      switch (state.nextMove)
	{
	case MOVE_RIGHT:
	  MouseReport->X = 1;
	  break;
	case MOVE_LEFT:
	  MouseReport->X = -1;
	  break;
	case MOVE_UP:
	  MouseReport->Y = -1;
	  break;
	case MOVE_DOWN:
	  MouseReport->Y = 1;
	  break;
	}
      if (state.moving-- == 0)
	{
	  state.lastMove = state.nextMove;
	  state.nextMove = MOVE_NONE;
	  TX_LedOff();
	}
    }
  
  *ReportSize = sizeof(USB_MouseReport_Data_t);
  return true;
}

/** HID class driver callback function for the processing of HID reports from the host.
 *
 *  \param[in] HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in] ReportID    Report ID of the received report from the host
 *  \param[in] ReportType  The type of report that the host has sent, either HID_REPORT_ITEM_Out or HID_REPORT_ITEM_Feature
 *  \param[in] ReportData  Pointer to a buffer where the received report has been stored
 *  \param[in] ReportSize  Size in bytes of the received HID report
 */
void CALLBACK_HID_Device_ProcessHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
                                          const uint8_t ReportID,
                                          const uint8_t ReportType,
                                          const void* ReportData,
                                          const uint16_t ReportSize)
{
  // Unused (but mandatory for the HID class driver) in this demo, since there are no Host->Device reports
}
/* EOF */
