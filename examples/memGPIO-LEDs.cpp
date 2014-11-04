/**
 * @file
 * Turn USRs LED on and off.
 *
 * Copyright (C) 2014 Manuel García (Manu@facine.es)
 */

#include "../src/memGPIO.hpp"

const int REPEATS = 10;
const int DELAY = 250000;

int main () {
  // Make a instance.
  easyBlack::memGPIO myExample;

  // Get PINs data for better performance.
  easyBlack::memGPIO::gpioPin usr0 = myExample.getPin ("USR0");
  easyBlack::memGPIO::gpioPin usr1 = myExample.getPin ("USR1");
  easyBlack::memGPIO::gpioPin usr2 = myExample.getPin ("USR2");
  easyBlack::memGPIO::gpioPin usr3 = myExample.getPin ("USR3");

  // Get PIN direction for better performance.
  const unsigned char output = myExample.OUTPUT;

  // Get PIN values for better performance.
  const unsigned char low = myExample.LOW;
  const unsigned char high = myExample.HIGH;

  // Set pin modes.
  myExample.pinMode (usr0, output);
  myExample.pinMode (usr1, output);
  myExample.pinMode (usr2, output);
  myExample.pinMode (usr3, output);

  for (int x = 0; x < REPEATS; x++) {
    // Turn USR0 LED on.
    myExample.digitalWrite (usr0, high);

    usleep (DELAY);

    // Turn USR0 LED off.
    myExample.digitalWrite (usr0, low);
    // Turn USR1 LED on.
    myExample.digitalWrite (usr1, high);

    usleep (DELAY);

    // Turn USR1 LED off.
    myExample.digitalWrite (usr1, low);
    // Turn USR2 LED on.
    myExample.digitalWrite (usr2, high);

    usleep (DELAY);

    // Turn USR2 LED off.
    myExample.digitalWrite (usr2, low);
    // Turn USR3 LED on.
    myExample.digitalWrite (usr3, high);

    usleep (DELAY);

    // Turn USR3 LED off.
    myExample.digitalWrite (usr3, low);

    usleep (DELAY);
  }

  // Reset all LED triggers to default.
  myExample.resetLEDPinsToDefault ();

  // Call the destructor.
  myExample.~memGPIO ();

  exit (EXIT_SUCCESS);
}
