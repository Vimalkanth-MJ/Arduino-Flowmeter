/*
 * arduino_io.h
 * From glcd library (https://github.com/johnmccombs/arduino-libraries/tree/master/glcd)
 *
 * this file maps arduino pins to avr ports and pins
 *
 * The header file: pins_arduino.h is used if this exits
 * otherwise the following controllers are defined in this file
 * Arduino (ATmega8,168,328), Mega, Sanguino (ATmega644P)
 * 
 * Thanks to Paul Stoffregen (http://www.pjrc.com/teensy) 
 * for his expertise in Arduino pin mapping macros
*/ 

#include "pins_arduino.h"
#if !(defined(digitalPinToPortReg) && defined(digitalPinToBit))
#if defined(__AVR_ATmega8__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)

  // Standard Arduino Pins
#define digitalPinToPortReg(P) \
    (((P) >= 0 && (P) <= 7) ? &PORTD : (((P) >= 8 && (P) <= 13) ? &PORTB : &PORTC))
#define digitalPinToBit(P) \
    (((P) >= 0 && (P) <= 7) ? (P) : (((P) >= 8 && (P) <= 13) ? (P) - 8 : (P) - 14))

#elif defined(__AVR_ATmega1280__)
  // Arduino Mega Pins
#define digitalPinToPortReg(P) \
    (((P) >= 22 && (P) <= 29) ? &PORTA : \
  ((((P) >= 10 && (P) <= 13) || ((P) >= 50 && (P) <= 53)) ? &PORTB : \
    (((P) >= 30 && (P) <= 37) ? &PORTC : \
  ((((P) >= 18 && (P) <= 21) || (P) == 38) ? &PORTD : \
  ((((P) >= 0 && (P) <= 3) || (P) == 5) ? &PORTE : \
    (((P) >= 54 && (P) <= 61) ? &PORTF : \
  ((((P) >= 39 && (P) <= 41) || (P) == 4) ? &PORTG : \
  ((((P) >= 6 && (P) <= 9) || (P) == 16 || (P) == 17) ? &PORTH : \
  (((P) == 14 || (P) == 15) ? &PORTJ : \
    (((P) >= 62 && (P) <= 69) ? &PORTK : &PORTL))))))))))
#define digitalPinToBit(P) \
  (((P) >=  7 && (P) <=  9) ? (P) - 3 : \
  (((P) >= 10 && (P) <= 13) ? (P) - 6 : \
  (((P) >= 22 && (P) <= 29) ? (P) - 22 : \
  (((P) >= 30 && (P) <= 37) ? 37 - (P) : \
  (((P) >= 39 && (P) <= 41) ? 41 - (P) : \
  (((P) >= 42 && (P) <= 49) ? 49 - (P) : \
  (((P) >= 50 && (P) <= 53) ? 53 - (P) : \
  (((P) >= 54 && (P) <= 61) ? (P) - 54 : \
  (((P) >= 62 && (P) <= 69) ? (P) - 62 : \
  (((P) == 0 || (P) == 15 || (P) == 17 || (P) == 21) ? 0 : \
  (((P) == 1 || (P) == 14 || (P) == 16 || (P) == 20) ? 1 : \
  (((P) == 19) ? 2 : \
  (((P) == 5 || (P) == 6 || (P) == 18) ? 3 : \
  (((P) == 2) ? 4 : \
  (((P) == 3 || (P) == 4) ? 5 : 7)))))))))))))))

#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644__)  
// Sanguino or other ATmega644 controller
#define digitalPinToPortReg(P) \   
    (((P) >= 0 && (P) <= 7)   ? &PORTB : \
  (((P) >= 8 && (P) <= 15)  ? &PORTD : \
  (((P) >= 16 && (P) <= 23) ? &PORTC : &PORTA)))
#define digitalPinToBit(P) \
     (((P) >= 0 && (P) <= 23) ? (P%8) : (7-(P%8)) )
//#error "ATmega644 has not been tested"
#elif defined(ESP32)
    // ESP32 Pins
#define digitalPinToPortReg(P) \
    (portOutputRegister(digitalPinToPort(P)))
#define digitalPinToBit(P) \
    (digitalPinToBitMask(P))

#elif defined(__AVR_ATmega32U4__)
    // Arduino Micro Pins
#define digitalPinToPortReg(P) \
    (((P) >= 0 && (P) <= 7) ? &PORTD : (((P) >= 8 && (P) <= 13) ? &PORTB : (((P) >= 14 && (P) <= 19) ? &PORTC : (((P) >= 20 && (P) <= 21) ? &PORTD : &PORTE)))))
#define digitalPinToBit(P) \
    (((P) >= 0 && (P) <= 7) ? (P) : (((P) >= 8 && (P) <= 13) ? (P) - 8 : (((P) >= 14 && (P) <= 19) ? (P) - 14 : (((P) >= 20 && (P) <= 21) ? (P) - 20 : (P) - 22)))))
#else 
#error "Arduino pin mapping not defined for this board"
#endif

#endif


#if !(defined(digitalWriteFast))
#define digitalWriteFast(pin, value) bitWrite(*digitalPinToPortReg(pin), digitalPinToBit(pin), value);
#endif
