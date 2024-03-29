/* This application converts the pedal of a Johannus Opus 1100N organ
 * to MIDI.  It uses a Teensy 2.0 microcontroller.
 *
 * The pedal is connected to the pedal through the cable that is
 * normally connected to CON5 on the organ's processor board.
 *
 *   PB0   1   2  PD3
 *   PB1   3   4  PD2
 *   PB2   5   6  PD1
 *   PB3   7   8  PD0
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

#define DEBOUNCE_TIME 50  // Number of cycles to debounce inputs
#define NPEDALS   32      // Number of pedals
#define BASE_NOTE 24      // C1, see https://www.inspiredacoustics.com/en/MIDI_note_numbers_and_center_frequencies
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
  // GlobalInterruptDisable();
  for (uint8_t block = 0x80; block; block >>= 1) {
    PORTB = ~block;
    _delay_us(100);
    state <<= 4;
    state |= (PIND & 0x0f);
  }
  // GlobalInterruptEnable();
  return ~state;
}

uint8_t midi_channel = 1;

void
send_midi_note(uint8_t note, uint8_t on)
{
  uint8_t command = on ? MIDI_COMMAND_NOTE_ON : MIDI_COMMAND_NOTE_OFF;
  MIDI_EventPacket_t MIDIEvent = (MIDI_EventPacket_t) {
    .Event       = MIDI_EVENT(0, command),
    .Data1       = command | MIDI_CHANNEL(midi_channel),
    .Data2       = note,
    .Data3       = MIDI_STANDARD_VELOCITY,
  };

  MIDI_Device_SendEventPacket(&Keyboard_MIDI_Interface, &MIDIEvent);
  MIDI_Device_Flush(&Keyboard_MIDI_Interface);
}

uint8_t scancode_to_keynum[32];

void
check_pedal(void)
{
  uint32_t pedal_state = read_pedal() | get_locked();
  uint32_t changed_state = pedal_state ^ last_pedal_state;
  uint32_t work_state = pedal_state;
  for (uint8_t scancode = 0; scancode < NPEDALS; scancode++) {
    if (changed_state & 1) {
      if (work_state & 1) {
        pedal_locks[scancode] = DEBOUNCE_TIME;
      }
      send_midi_note(BASE_NOTE + scancode_to_keynum[scancode], work_state & 1);
    }
    changed_state >>= 1;
    work_state >>= 1;
  }
  last_pedal_state = pedal_state;
}

void
flush_midi_receive_events(void)
{
    MIDI_EventPacket_t ReceivedMIDIEvent;
  
    while (MIDI_Device_ReceiveEventPacket(&Keyboard_MIDI_Interface, &ReceivedMIDIEvent)) {
    }
}

void
init_midi_channel(void)
{
  uint8_t new_midi_channel = midi_channel = eeprom_read_byte((uint8_t*) 0);
  uint32_t pedal_state = read_pedal();
  if (pedal_state) {
    for (uint8_t scancode = 0; scancode < NPEDALS; scancode++, pedal_state >>= 1) {
      if (pedal_state & 1 && scancode_to_keynum[scancode] != 255) {
        new_midi_channel = scancode_to_keynum[scancode];
      }
    }
  }
  if (new_midi_channel < 1 || new_midi_channel > 15) {
    new_midi_channel = 1;
  }
  if (midi_channel != new_midi_channel) {
    eeprom_write_byte((uint8_t*) 0, new_midi_channel);
  }
  midi_channel = new_midi_channel;
}

void
init_scancode_map(void)
{
  // establish mapping from scan code to key number
  scancode_to_keynum[28] = 255;
  scancode_to_keynum[24] = 255;
  scancode_to_keynum[20] = 29;
  scancode_to_keynum[16] = 28;
  scancode_to_keynum[12] = 27;
  scancode_to_keynum[8] = 26;
  scancode_to_keynum[4] = 25;
  scancode_to_keynum[0] = 24;
  scancode_to_keynum[29] = 23;
  scancode_to_keynum[25] = 22;
  scancode_to_keynum[21] = 21;
  scancode_to_keynum[17] = 20;
  scancode_to_keynum[13] = 19;
  scancode_to_keynum[9] = 18;
  scancode_to_keynum[5] = 17;
  scancode_to_keynum[1] = 16;
  scancode_to_keynum[30] = 15;
  scancode_to_keynum[26] = 14;
  scancode_to_keynum[22] = 13;
  scancode_to_keynum[18] = 12;
  scancode_to_keynum[14] = 11;
  scancode_to_keynum[10] = 10;
  scancode_to_keynum[6] = 9;
  scancode_to_keynum[2] = 8;
  scancode_to_keynum[31] = 7;
  scancode_to_keynum[27] = 6;
  scancode_to_keynum[23] = 5;
  scancode_to_keynum[19] = 4;
  scancode_to_keynum[15] = 3;
  scancode_to_keynum[11] = 2;
  scancode_to_keynum[7] = 1;
  scancode_to_keynum[3] = 0;
}

int
main(void)
{
  SetupHardware();

  GlobalInterruptEnable();

  init_scancode_map();

  init_midi_channel();

  for (;;) {
    check_pedal();
    if (sys_led_state) {
      PORTD |= SYS_LED_MASK;
    } else {
      PORTD &= ~SYS_LED_MASK;
    }

    flush_midi_receive_events();
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
  DDRB = 0xFF;
  PORTB = 0xFF;
  
  DDRD = 0x40;
  PORTD |= 0x0F; // pull-ups aktivieren

  TCCR0A = 0x00;                                // Normal Counter mode
  TCCR0B |= (1<<CS01)|(1<<CS00);                // prescaler 64
  TCNT0 = 6;                                    // Vorladen mit 6, damit timer 250 Tyktzyklen zaehlt
  TIMSK0 = 0x01;                                // Overflow Interrupt enablen

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

