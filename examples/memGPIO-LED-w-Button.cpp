/**
 * @file
 * Use a pushbutton to control an USR LED.
 *
 * Copyright (C) 2014 Manuel García (Manu@facine.es)
 */

#include "../src/memGPIO.hpp"

int main () {
  // Make a instance.
  easyBlack::memGPIO myExample;

  // Get PINs data for better performance.
  easyBlack::memGPIO::gpioPin usr0 = myExample.getPin ("USR0");
  easyBlack::memGPIO::gpioPin p8_7 = myExample.getPin ("P8_7");

  // Get PIN direction for better performance.
  const unsigned char output = myExample.OUTPUT;
  const unsigned char input = myExample.INPUT;

  // Get PIN values for better performance.
  const unsigned char low = myExample.LOW;
  const unsigned char high = myExample.HIGH;

  // Variable for reading the pushbutton status.
  unsigned char buttonState = low;

  // Set pin modes.
  myExample.pinMode (usr0, output);
  myExample.pinMode (p8_7, input);

  while (1) {
    // Read the state of the pushbutton value.
    buttonState = myExample.digitalRead (usr0);

    // Check if the pushbutton is pressed.
    if (buttonState == high) {
      // Turn USR0 LED on.
      myExample.digitalWrite (usr0, high);
    }
    else {
      // Turn USR0 LED off.
      myExample.digitalWrite (usr0, low);
    }
  }

  // Reset USR0 LED trigger to default.
  myExample.resetLEDPin0ToDefault ();

  // Call the destructor.
  myExample.~memGPIO ();

  exit (EXIT_SUCCESS);
}
