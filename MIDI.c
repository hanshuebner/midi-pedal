/* This application converts the pedal of a Johannus Opus 1100N organ
 * to MIDI.  It uses a Teensy 2.0 microcontroller.
 *
 * The pedal is connected to the pedal through the cable that is
 * normally connected to CON5 on the organ's processor board.
 *
 *   PB0   1   2  PD0
 *   PB1   3   4  PD1
 *   PB2   5   6  PD2
 *   PB3   7   8  PD3
 *   PB4   9  10
 *   PB5  11  12
 *   PB6  13  14
 *   PB7  15  16
 *
 */

#include "MIDI.h"


USB_ClassInfo_MIDI_Device_t Keyboard_MIDI_Interface = {
  .Config = {
    .StreamingInterfaceNumber = INTERFACE_ID_AudioStream,
    .DataINEndpoint = {
      .Address          = MIDI_STREAM_IN_EPADDR,
      .Size             = MIDI_STREAM_EPSIZE,
      .Banks            = 1,
    },
    .DataOUTEndpoint = {
      .Address          = MIDI_STREAM_OUT_EPADDR,
      .Size             = MIDI_STREAM_EPSIZE,
      .Banks            = 1,
    },
  },
};

#define DEBOUNCE_TIME 10 // Number of 2ms cycles to debounce inputs
#define NPEDALS   32     // Number of pedals
#define BASE_NOTE 24     // C1, see https://www.inspiredacoustics.com/en/MIDI_note_numbers_and_center_frequencies
static uint8_t pedal_locks[NPEDALS];
static uint32_t last_pedal_state;

#define SYS_LED_MASK 0x40
static uint8_t sys_led_cycle = 0;
static uint8_t sys_led_state = 0;

ISR (TIMER0_OVF_vect)
{
  for (uint8_t i = 0; i < NPEDALS; i++) {
    if (pedal_locks[i]) {
      pedal_locks[i]--;
    }
  }
  sys_led_state = (sys_led_cycle++ & 0x80) ? SYS_LED_MASK : 0;
}

uint32_t
get_locked(void)
{
  uint32_t locked = 0;
  GlobalInterruptDisable();
  for (uint8_t i = 0; i < NPEDALS; i++) {
    locked >>= 1;
    if (pedal_locks[i]) {
      locked |= 0x80000000;
    }
  }
  GlobalInterruptEnable();
  return locked;
}

uint32_t
read_pedal(void)
{
  uint32_t state = 0;
  GlobalInterruptDisable();
  for (uint8_t block = 0x08; block; block >>= 1) {
    PORTD = (block ^ 0x0F) | sys_led_state;
    state <<= 8;
    state |= PINB;
  }
  GlobalInterruptEnable();
  if (~state) {
    PORTD |= SYS_LED_MASK;
  }
  return ~state;
}

void
check_pedal(void)
{
  uint32_t pedal_state = read_pedal() | get_locked();
  uint32_t changed_state = pedal_state ^ last_pedal_state;
  uint32_t work_state = pedal_state;
  uint8_t note = BASE_NOTE;
  for (uint8_t i = 0; i < NPEDALS; i++) {
    if (changed_state & 1) {
      if (work_state & 1) {
        pedal_locks[i] = DEBOUNCE_TIME;
      }

      uint8_t command = (work_state & 1) ? MIDI_COMMAND_NOTE_ON : MIDI_COMMAND_NOTE_OFF;
      MIDI_EventPacket_t MIDIEvent = (MIDI_EventPacket_t) {
        .Event       = MIDI_EVENT(0, command),
        .Data1       = command | MIDI_CHANNEL(1),
        .Data2       = note,
        .Data3       = MIDI_STANDARD_VELOCITY,
      };

      MIDI_Device_SendEventPacket(&Keyboard_MIDI_Interface, &MIDIEvent);
      MIDI_Device_Flush(&Keyboard_MIDI_Interface);
    }
    changed_state >>= 1;
    work_state >>= 1;
    note++;
  }
  last_pedal_state = pedal_state;
}

int
main(void)
{
  SetupHardware();

  GlobalInterruptEnable();

  for (;;) {
    MIDI_EventPacket_t ReceivedMIDIEvent;
    check_pedal();

    while (MIDI_Device_ReceiveEventPacket(&Keyboard_MIDI_Interface, &ReceivedMIDIEvent)) {
#if 0
      if ((ReceivedMIDIEvent.Event == MIDI_EVENT(0, MIDI_COMMAND_NOTE_ON)) && (ReceivedMIDIEvent.Data3 > 0))
        LEDs_SetAllLEDs(ReceivedMIDIEvent.Data2 > 64 ? LEDS_LED1 : LEDS_LED2);
      else
        LEDs_SetAllLEDs(LEDS_NO_LEDS);
#endif
    }

    MIDI_Device_USBTask(&Keyboard_MIDI_Interface);
    USB_USBTask();
  }
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
  /* Disable watchdog if enabled by bootloader/fuses */
  MCUSR &= ~(1 << WDRF);
  wdt_disable();

  /* Disable clock division */
  clock_prescale_set(clock_div_1);

  /* Hardware Initialization */
  USB_Init();

  // Ports initialisieren
  DDRB = 0;
  PORTB = 0xFF; // pull-ups aktivieren
  
  DDRD = 0x4F;
  PORTD |= 0x0F;

  TCCR0A=0x00;                  // Normal Counter mode
  TCCR0B|=(1<<CS01)|(1<<CS00);  // prescaler 64
  TCNT0=6;                     // Vorladen mit 6, damit timer 250 Tyktzyklen zaehlt
  TIMSK0=0x01;                 // Overflow Interrupt enablen
#if 0
  // Timer 0 konfigurieren
  TCCR0A = (1<<CS01); // Prescaler 8

  // Overflow Interrupt erlauben
  TIMSK0 |= (1<<TOIE0);
#endif
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
  bool ConfigSuccess = true;

  ConfigSuccess &= MIDI_Device_ConfigureEndpoints(&Keyboard_MIDI_Interface);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
  MIDI_Device_ProcessControlRequest(&Keyboard_MIDI_Interface);
}

