easyBlack
=========

Beaglebone Black C++ Library for GPIO using high performance of mmap.

PreRequisites
-------------
Debian and Ubuntu:
````sh
sudo apt-get install -y build-essential g++
wget -c https://raw.github.com/RobertCNelson/tools/master/pkgs/dtc.sh
chmod +x dtc.sh
sudo ./dtc.sh
````

Example
-------
See more examples en examples directory.
````cpp
#include "memGPIO.hpp"

const int REPEATS = 10;
const int DELAY = 250000;

int main () {
  // Make a instance.
  easyBlack::memGPIO myExample;

  // Get PINs data for better performance.
  easyBlack::memGPIO::gpioPin usr0 = myExample.getPin ("USR0");

  // Get PIN direction for better performance.
  const unsigned char output = myExample.OUTPUT;

  // Get PIN values for better performance.
  const unsigned char low = myExample.LOW;
  const unsigned char high = myExample.HIGH;

  // Set pin mode.
  myExample.pinMode (usr0, output);

  for (int x = 0; x < REPEATS; x++) {
    // Turn USR0 LED on.
    myExample.digitalWrite (usr0, high);

    usleep (DELAY);

    // Turn USR0 LED off.
    myExample.digitalWrite (usr0, low);

    usleep (DELAY);
  }

  // Reset USR0 LED trigger to default.
  myExample.resetLEDPin0ToDefault ();

  // Call the destructor.
  myExample.~memGPIO ();

  exit (EXIT_SUCCESS);
}
```

Digital and Analog I/O
----------------------
* getPin (pinNumber)
* pinMode (pin, direction, [mux], [pullup], [slew])
* digitalRead (pin)
* digitalWrite (pin, value)
* resetLEDPin0ToDefault ()
* resetLEDPin1ToDefault ()
* resetLEDPin2ToDefault ()
* resetLEDPin3ToDefault ()
* resetLEDPinsToDefault ()

License
=======

    The MIT License (MIT)

    Copyright (c) 2014 facine

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
