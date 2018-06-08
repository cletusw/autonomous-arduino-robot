const int8_t encoder_pci_lookup_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};

// Pins must be adjacent and in same pin change interrupt port
const uint8_t encoderDPinA = 7;
const uint8_t encoderDPinB = 6;
// Number of bit positions to shift PIND right to get above pins into least-significant bits, ie 0b00xx
// To find out, log it or check https://www.arduino.cc/en/Hacking/Atmega168Hardware
// (168 is same as 328 for the Uno). Look for PDx where x is the bit position, 0 being LSB
const uint8_t encoderDShift = 6;
volatile long encoderDCountInternal = 0;

const uint8_t encoderBPinA = 8;
const uint8_t encoderBPinB = 9;
// Number of bit positions to shift PINB right to get above pins into least-significant bits, ie 0b00xx
// To find out, log it or check https://www.arduino.cc/en/Hacking/Atmega168Hardware
// (168 is same as 328 for the Uno). Look for PBx where x is the bit position, 0 being LSB
const uint8_t encoderBShift = 0;
volatile long encoderBCountInternal = 0;

void pciSetup(byte pin) {
  *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
  PCIFR |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
  PCICR |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

void setUpEncoderPins() {
  pinMode(encoderDPinA, INPUT_PULLUP);
  pinMode(encoderDPinB, INPUT_PULLUP);
  pinMode(encoderBPinA, INPUT_PULLUP);
  pinMode(encoderBPinB, INPUT_PULLUP);

  noInterrupts();
  pciSetup(encoderDPinA);
  pciSetup(encoderDPinB);
  pciSetup(encoderBPinA);
  pciSetup(encoderBPinB);
  interrupts();
}

// handle pin change interrupt for D0 to D7, for encoderD
// See http://makeatronics.blogspot.com/2013/02/efficiently-reading-quadrature-with.html
ISR (PCINT2_vect) {
  static uint8_t enc_val = 0;

  enc_val = enc_val << 2; // Make room for new bits
  enc_val = enc_val | ((PIND >> encoderDShift) & 0b11); // PIND is pins 0-7

  encoderDCountInternal -= encoder_pci_lookup_table[enc_val & 0b1111];
}

// handle pin change interrupt for D8 to D13, for encoderB
// See http://makeatronics.blogspot.com/2013/02/efficiently-reading-quadrature-with.html
ISR (PCINT0_vect) {
  static uint8_t enc_val = 0;

  enc_val = enc_val << 2; // Make room for new bits
  enc_val = enc_val | ((PINB >> encoderBShift) & 0b11); // PINB is pins 8-13

  encoderBCountInternal -= encoder_pci_lookup_table[enc_val & 0b1111];
}


