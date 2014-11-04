/**
 * @file
 * Headers file of BeagleBone Black GPIO library.
 *
 * Copyright (C) 2014 Manuel García (Manu@facine.es)
 */

#ifndef _MEMGPIO_HPP_
#define _MEMGPIO_HPP_

#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/utsname.h>
#include "utils.hpp"

namespace easyBlack {
  class memGPIO {
    public:
      /**
       * Typedef struct for PWWM pins.
       *
       * @member std::string module
       *   PWM module.
       * @member char sysfs
       *   PWM sysfs.
       * @member char index
       *   PWM index.
       * @member char muxMode
       *   Mux mode value, 0-7. See am335x technical manual.
       * @member std::string path
       *   PWM pin path.
       * @member std::string name
       *   PWM pin name.
       */
      typedef struct {
        std::string module;
        unsigned char sysfs;
        unsigned char index;
        unsigned char muxMode;
        std::string path;
        std::string name;
      } gpioPwm;

      /**
       * Typedef struct for PIN modes.
       *
       * The mode name for that pin in the mode being used.
       */
      typedef struct {
        std::string mode0;
        std::string mode1;
        std::string mode2;
        std::string mode3;
        std::string mode4;
        std::string mode5;
        std::string mode6;
        std::string mode7;
      } gpioModes;

      /**
       * Typedef struct for PINs.
       *
       * @member std::string name
       *   The readable pin name. See beaglebone user guide.
       * @member std::string key
       *   The pin number identifier, i.e.: P8_3.
       * @member std::string led
       *   If a pin has on-board LED.
       * @member unsigned int gpioBank
       *   Wich of the four GPIO bank is the pin in.
       * @member unsigned char gpioPin
       *   Pin number on the am335x processor.
       * @member unsigned char bankId
       *   Pin number within each bank, 0-31.
       * @member unsigned char eeprom
       *   Position in EEPROM
       * @member std::string mux
       *   Setting mux file name.
       * @member unsigned int muxRegOffset
       *   The control module register offset from 0x44E10800 as used in the
       *   device tree.
       * @member unsigned char ain
       *   The ain value, only for analog pins.
       * @member unsigned short int scale
       *   The scale value, only for analog pins.
       * @member gpioPwm pwm
       *   PWM struct if pwmPresent is TRUE.
       * @member gpioModes modes
       *   The pin modes.
       */
      typedef struct {
        std::string name;
        std::string key;
        std::string led;
        unsigned int gpioBank;
        unsigned char gpioPin;
        unsigned char bankId;
        unsigned char eeprom;
        std::string mux;
        unsigned int muxRegOffset;
        unsigned char ain;
        unsigned short int scale;
        gpioPwm pwm;
        gpioModes modes;
      } gpioPin;

      // Pin directions.
      static const unsigned char OUTPUT = 0;
      static const unsigned char INPUT = 1;
      static const unsigned char INPUT_PULLUP = 2;

      // Pin values.
      static const unsigned char LOW = 0;
      static const unsigned char HIGH = 1;

      // Pull modes.
      static const unsigned char PULLDOWN = 0;
      static const unsigned char PULLUP = 1;
      static const unsigned char DISABLED = 2;

      // Slew values.
      static const unsigned char SLOW = 0;
      static const unsigned char FAST = 1;

      /**
       * Constructor: check kernel version, mmaps into /dev/mem and enable
       * GPIO / EHRPWM / ADC module clocks.
       */
      memGPIO ();

      /**
       * Destructor: removes mapping between process memory and physical memory.
       */
      ~memGPIO ();

      /**
       * Reads the value from the specified analog pin.
       *
       * @param const gpioPin pin
       *   The pin you want to read.
       *
       * @return int
       *   The analog value of pin.
       */
//      int analogRead (const gpioPin pin);

      /**
       * Reads the value from the specified analog pin.
       *
       * @param std::string pinNumber
       *   The pin you want to read.
       *
       * @return int
       *   The analog value of pin.
       */
//      int analogRead (std::string pinNumber);

      /**
       * Reads the value from a specified digital pin, either HIGH or LOW.
       *
       * @param const gpioPin pin
       *   The pin you want to read.
       *
       * @return unsigned char
       *   Can return either HIGH or LOW data values.
       */
      unsigned char digitalRead (const gpioPin pin);

      /**
       * Reads the value from a specified digital pin, either HIGH or LOW.
       *
       * @param std::string pinNumber
       *   The number of the digital pin you want to read.
       *
       * @return unsigned char
       *   Can return either HIGH or LOW data values.
       */
      unsigned char digitalRead (std::string pinNumber);

      /**
       *  Write a HIGH or a LOW value to a digital pin.
       *
       * @param const gpioPin pin
       *   The pin whose you wish to write data.
       * @param const unsigned char pinValue
       *   HIGH or LOW values.
       */
      void digitalWrite (const gpioPin pin, const unsigned char pinValue);

      /**
       *  Write a HIGH or a LOW value to a digital pin.
       *
       * @param std::string pinNumber
       *   The number of the pin whose you wish to write data.
       * @param std::string value
       *   HIGH or LOW values.
       */
      void digitalWrite (std::string pinNumber, std::string value);

      /**
       *  Get pin from pin number.
       *
       * @param std::string pinNumber
       *   The number of the pin you want to get.
       *
       * @return const gpioPin
       *   The pin data.
       */
      const gpioPin getPin (std::string pinNumber);

      /**
       * Configures the specified pin to behave either as an input or an output.
       *
       * @param const gpioPin pin
       *   The number of the pin whose mode you wish to set.
       * @param const unsigned char direction
       *   INPUT or OUTPUT modes.
       * @param unsigned char mux
       *   Index to mux mode (under development on 3.8 kernel). Default: 7.
       * @param unsigned char pullup
       *   'PULLUP', 'PULLDOWN' or 'DISABLED' (under development on 3.8 kernel).
       *   Default: DISABLED.
       * @param unsigned char slew
       *   'FAST' or 'SLOW' (under development on 3.8 kernel). Default: FAST.
       */
      void pinMode (const gpioPin pin, const unsigned char direction, unsigned char mux = 7, unsigned char pullup = DISABLED, unsigned char slew = FAST);

      /**
       * Configures the specified pin to behave either as an input or an output.
       *
       * @param std::string pinNumber
       *   The number of the pin whose mode you wish to set.
       * @param std::string pinDirection
       *   INPUT or OUTPUT modes.
       * @param unsigned char pinMux
       *   Index to mux mode (under development on 3.8 kernel). Default: 7.
       * @param std::string pinPullup
       *   'PULLUP', 'PULLDOWN' or 'DISABLED' (under development on 3.8 kernel).
       *   Default: DISABLED.
       * @param std::string pinSlew
       *   'FAST' or 'SLOW' (under development on 3.8 kernel). Default: FAST.
       */
      void pinMode (std::string pinNumber, std::string pinDirection, unsigned char pinMux = 7, std::string pinPullup = "DISABLED", std::string pinSlew = "FAST");

      /**
       *  Reset USR0 LED pin trigger to default.
       */
      void resetLEDPin0ToDefault ();

      /**
       *  Reset USR1 LED pin trigger to default.
       */
      void resetLEDPin1ToDefault ();

      /**
       *  Reset USR2 LED pin trigger to default.
       */
      void resetLEDPin2ToDefault ();

      /**
       *  Reset USR3 LED pin trigger to default.
       */
      void resetLEDPin3ToDefault ();

      /**
       *  Reset all LED pin triggers to default.
       */
      void resetLEDPinsToDefault ();

    private:
      // MMAP memory registers.
      static const unsigned int MMAP_OFFSET = 0x44C00000;
      static const unsigned int MMAP_SIZE = 0x481AEFFF - MMAP_OFFSET;

      // GPIO memory whole 4-byte register.
      static const unsigned int GPIO_REGISTER_SIZE = 4;

      // GPIO memory registers.
      static const unsigned int GPIO_BASE = 0x44e10800;
      static const unsigned int GPIO0 = 0x44E07000;
      static const unsigned int GPIO1 = 0x4804C000;
      static const unsigned int GPIO2 = 0x481AC000;
      static const unsigned int GPIO3 = 0x481AE000;

      static const unsigned int GPIO_OE = 0x134;
      static const unsigned int GPIO_DATAOUT = 0x13C;
      static const unsigned int GPIO_DATAIN = 0x138;

      // Clock Module registers.
      static const unsigned int MODULEMODE_ENABLE = 0x02;
      static const unsigned int IDLEST_MASK = (0x03 << 16);

      static const unsigned int CM_WKUP = 0x44E00400;
      static const unsigned int CM_PER = 0x44E00000;

      static const unsigned int CM_WKUP_GPIO0_CLKCTRL = (CM_WKUP + 0x8);
    
      static const unsigned int CM_PER_GPIO1_CLKCTRL = (CM_PER + 0xAC);
      static const unsigned int CM_PER_GPIO2_CLKCTRL = (CM_PER + 0xB0);
      static const unsigned int CM_PER_GPIO3_CLKCTRL = (CM_PER + 0xB4);

      static const unsigned int CM_PER_EPWMSS0_CLKCTRL = (CM_PER + 0xD4);
      static const unsigned int CM_PER_EPWMSS1_CLKCTRL = (CM_PER + 0xCC);
      static const unsigned int CM_PER_EPWMSS2_CLKCTRL = (CM_PER + 0xD8);

      static const unsigned int ADC_TSC = 0x44E0D000;
      static const unsigned int ADC_SYSCONFIG = (ADC_TSC + 0x10);
      static const unsigned int ADC_SOFTRESET = 0x01;
      static const unsigned int ADC_STEPCONFIG_WRITE_PROTECT_OFF = (0x01 << 2);
      static const unsigned int ADC_STEPENABLE = (ADC_TSC + 0x54);
      static const unsigned int TSC_ADC_SS_ENABLE = 0x01;
      static const unsigned int ADC_CTRL = (ADC_TSC + 0x40);
      static const unsigned int CM_WKUP_ADC_TSC_CLKCTRL = (CM_WKUP + 0xBC);
      static const unsigned int ADC_FIFO0DATA = (ADC_TSC + 0x100);
      static const unsigned int ADC_FIFO_MASK = 0xFFF;

      // ADC STEPCONFIG.
      static const unsigned int ADCSTEPDELAY = ((15 & 0xFF) << 24);

      static const unsigned int ADCSTEPCONFIG1 = (ADC_TSC + 0x64);
      static const unsigned int ADCSTEPCONFIG1_VALUE = (0X00 << 19) | (0x01 << 2);
      static const unsigned int ADCSTEPDELAY1 = (ADC_TSC + 0x68);

      static const unsigned int ADCSTEPCONFIG2 = (ADC_TSC + 0x6C);
      static const unsigned int ADCSTEPCONFIG2_VALUE = (0X01 << 19) | (0x01 << 2);
      static const unsigned int ADCSTEPDELAY2 = (ADC_TSC + 0x70);

      static const unsigned int ADCSTEPCONFIG3 = (ADC_TSC + 0x74);
      static const unsigned int ADCSTEPCONFIG3_VALUE = (0X02 << 19) | (0x01 << 2);
      static const unsigned int ADCSTEPDELAY3 = (ADC_TSC + 0x78);

      static const unsigned int ADCSTEPCONFIG4 = (ADC_TSC + 0x7C);
      static const unsigned int ADCSTEPCONFIG4_VALUE = (0X03 << 19) | (0x01 << 2);
      static const unsigned int ADCSTEPDELAY4 = (ADC_TSC + 0x80);
    
      static const unsigned int ADCSTEPCONFIG5 = (ADC_TSC + 0x84);
      static const unsigned int ADCSTEPCONFIG5_VALUE = (0X04 << 19) | (0x01 << 2);
      static const unsigned int ADCSTEPDELAY5 = (ADC_TSC + 0x88);
    
      static const unsigned int ADCSTEPCONFIG6 = (ADC_TSC + 0x8C);
      static const unsigned int ADCSTEPCONFIG6_VALUE = (0X05 << 19) | (0x01 << 2);
      static const unsigned int ADCSTEPDELAY6 = (ADC_TSC + 0x90);
    
      static const unsigned int ADCSTEPCONFIG7 = (ADC_TSC + 0x94);
      static const unsigned int ADCSTEPCONFIG7_VALUE = (0X06 << 19) | (0x01 << 2);
      static const unsigned int ADCSTEPDELAY7 = (ADC_TSC + 0x98);
    
      static const unsigned int ADCSTEPCONFIG8 = (ADC_TSC + 0x9C);
      static const unsigned int ADCSTEPCONFIG8_VALUE = (0X07 << 19) | (0x01 << 2);
      static const unsigned int ADCSTEPDELAY8 = (ADC_TSC + 0xA0);

      // Kernel version.
      static unsigned char oldKernel;

      // Available PINs.
      static const std::map <std::string, const gpioPin> PIN_INDEX;

      // DTS file templates.
      static const std::string BSPM_TEMPLATE;
      static const std::string BSPWM_TEMPLATE;

      // MMAP Address pointer.
      static volatile unsigned long int *mapAddress;

      /**
       *  Check the current kernel version.
       */
      void checkKernelVersion ();

      /**
       *  Compose the DTS file for the PIN.
       *
       * @param const gpioPin pin
       *   The pin whose you wish to write data.
       * @param int pinData
       *   The pin data.
       * @param std::string pinTemplate
       *   The pin template, default "bspm".
       *
       * @return std::string
       *   Return the composed DTS for the PIN.
       */
      std::string composeDTS (const gpioPin pin, int pinData, std::string pinTemplate = "bspm");

      /**
       *  Create the DTBO file for the PIN.
       *
       * @param std::string dtboFilename
       *   The DTBO file name.
       * @param std::string dtsFilename
       *   The DTS file name.
       */
      void createDTBO (std::string dtboFilename, std::string dtsFilename);

      /**
       *  Create the DTS file for the PIN.
       *
       * @param const gpioPin pin
       *   The pin whose you wish to write data.
       * @param int pinData
       *   The pin data.
       * @param std::string dtsFilename
       *   The DTS file name.
       * @param std::string pinTemplate
       *   The pin template, default "bspm".
       */
      void createDTS (const gpioPin pin, int pinData, std::string dtsFilename, std::string pinTemplate = "bspm");

      /**
       * Enable clock modules of each GPIO.
       */
      void enableClockModules ();

      /**
       * Figure out the desired value to the pin.
       *
       * @param const unsigned char direction
       *   INPUT or OUTPUT modes.
       * @param unsigned char mux
       *   Index to mux mode (under development on 3.8 kernel).
       * @param unsigned char pullup
       *   'PULLUP', 'PULLDOWN' or 'DISABLED' (under development on 3.8 kernel).
       * @param unsigned char slew
       *   'FAST' or 'SLOW' (under development on 3.8 kernel).
       *
       * @return int
       *   The pin data value.
       */
      int figureOutPinData (const unsigned char direction, unsigned char mux, unsigned char pullup, unsigned char slew);

      /**
       *  Get the pin mode value.
       *
       * @param std::string pinDirection
       *   Direction to figure out the value. INPUT or OUTPUT.
       *
       * @return const unsigned char
       *   Mode value.
       */
      const unsigned char getPinDirection (std::string pinDirection);

      /**
       *  Get the pin mode value.
       *
       * @param unsigned char pinMode
       *   The GPIO pin mode number.
       *
       * @return std::string
       *   The GPIO pin mode value.
       */
      std::string getPinMode (const gpioPin pin, unsigned char pinMode);

      /**
       *  Get the pin value.
       *
       * @param std::string pinValue
       *   Value to figure out the value. LOW or HIGH.
       *
       * @return const unsigned char
       *   Value value.
       */
      const unsigned char getPinValue (std::string pinValue);

      /**
       *  Get the pull mode value.
       *
       * @param std::string pullMode
       *   Pull mode to figure out the value. PULLUP, PULLDOWN or DISABLED.
       *
       * @return const unsigned char
       *   Pull mode value.
       */
      const unsigned char getPullMode (std::string pullMode);

      /**
       *  Get the slew value.
       *
       * @param std::string slewValue
       *   Selew value to figure out the value. FAST or SLOW.
       *
       * @return const unsigned char
       *    Selew value value.
       */
      const unsigned char getSlewValue (std::string slewValue);

      /**
       *  Initialize BSPM Template.
       *
       * @return sconst std::string
       *   A DTS Template string.
       */
      static const std::string initializeBSPMTemplate ();

      /**
       *  Initialize BSPM Template.
       *
       * @return sconst std::string
       *   A DTS Template string.
       */
      static const std::string initializeBSPWMTemplate ();

      /**
       *  Initialize Pin values.
       *
       * @return std::map <std::string, const gpioPin>
       *   A map with all PIN values keyered by pin key.
       */
      static std::map <std::string, const gpioPin> initializePins ();

      /**
       *  Mmaps into /dev/mem.
       */
      void mapsMemory ();

      /**
       *  Set LED pin to default trigger.
       *
       * @param const gpioPin pin
       *   The number of the pin whose trigger you wish reset.
       */
      void setLEDPinToDefault (const gpioPin pin);

      /**
       *  Set LED pin to GPIO trigger.
       *
       * @param const gpioPin pin
       *   The number of the pin whose trigger you wish to set to gpio.
       */
      void setLEDPinToGPIO (const gpioPin pin);

      /**
       * Sets the specified pin data.
       *
       * @param const gpioPin pin
       *   The number of the pin whose mode you wish to set.
       * @param int pinData
       *   The pin data.
       * @param std::string pinTemplate
       *   The pin template, default "bspm".
       */
      void setPinMode (const gpioPin pin, int pinData, std::string pinTemplate = "bspm");

      /**
       * Unload the specified driver.
       *
       * @param const std::string slotsFile
       *   The slots file path.
       * @param const std::string fragment
       *   The driver to search.
       */
      void unloadSlot (const std::string slotsFile, const std::string fragment);
  };
}
#endif
