/**
 * @file
 * BeagleBone Black GPIO library.
 *
 * Copyright (C) 2014 Manuel García (Manu@facine.es)
 *
 * This C++ library uses the Linux mmap function to execute GPIO operations,
 * where values are read and written directly from system memory.
 */

#include "memGPIO.hpp"

namespace easyBlack {
  // Register and initialize variables that can be accessed as a part of the
  // class.
  const unsigned char memGPIO::OUTPUT;
  const unsigned char memGPIO::INPUT;
  const unsigned char memGPIO::INPUT_PULLUP;

  const unsigned char memGPIO::PULLDOWN;
  const unsigned char memGPIO::PULLUP;
  const unsigned char memGPIO::DISABLED;

  const unsigned char memGPIO::SLOW;
  const unsigned char memGPIO::FAST;

  const unsigned char memGPIO::LOW;
  const unsigned char memGPIO::HIGH;

  unsigned char memGPIO::oldKernel;

  const std::map<std::string, const memGPIO::gpioPin> memGPIO::PIN_INDEX = memGPIO::initializePins ();

  const std::string memGPIO::BSPM_TEMPLATE= memGPIO::initializeBSPMTemplate ();
  const std::string memGPIO::BSPWM_TEMPLATE= memGPIO::initializeBSPWMTemplate ();

  volatile unsigned long int *memGPIO::mapAddress = nullptr;

  /**
   * Public methods.
   */
  memGPIO::memGPIO () {
    // Check if kernel version is < 3.8
    checkKernelVersion ();
    // Mmaps into /dev/mem.
    mapsMemory ();
    // Enable all GPIO / EHRPWM / ADC module clocks.
    enableClockModules ();
  }

  memGPIO::~memGPIO () {
    if (munmap ((void *) mapAddress, MMAP_SIZE) < 0) {
      perror ("munmap (gpio) failed");
      exit (EXIT_FAILURE);
    }
  }

/*  int memGPIO::analogRead (const memGPIO::gpioPin pin) {
    if (pin.name.compare (0, 3, "AIN") == 0) {
      std::cout << "Invalid analog pin: " << pin.key << "!" << std::endl;
      exit (EXIT_FAILURE);
    }

    // Enable sequncer step that's set for given input.
    mapAddress[(ADC_STEPENABLE - MMAP_OFFSET) / GPIO_REGISTER_SIZE] |= (0x01 << (pin.bankId + 1));
    // Sequencer starts automatically after enabling step, wait for complete.
    while (mapAddress[(ADC_STEPENABLE - MMAP_OFFSET) / GPIO_REGISTER_SIZE] & (0x01 << (pin.bankId + 1)));

    // Return 12-bit value from the ADC FIFO register.
    return mapAddress[(ADC_FIFO0DATA - MMAP_OFFSET) / GPIO_REGISTER_SIZE] & ADC_FIFO_MASK;
  }

  int memGPIO::analogRead (std::string pinNumber) {
    // Get the gpioPin from pin key.
    gpioPin pin = getPin (pinNumber);
    
    // Read the value for that pin and return it.
    return analogRead (pin);
  }*/

  unsigned char memGPIO::digitalRead (const memGPIO::gpioPin pin) {
    // Read the value for this pin.
    unsigned char value = (mapAddress[(pin.gpioBank - MMAP_OFFSET + GPIO_DATAIN) / GPIO_REGISTER_SIZE] & (1 << pin.bankId)) >> pin.bankId;

    return value;
  }

  unsigned char memGPIO::digitalRead (std::string pinNumber) {
    // Get the gpioPin from pin key.
    gpioPin pin = getPin (pinNumber);

    // Read the value for that pin and return it.
    return digitalRead (pin);
  }

  void memGPIO::digitalWrite (const memGPIO::gpioPin pin, const unsigned char value) {
    switch (value) {
      case LOW:
        // Set LOW value to this pin.
        mapAddress[(pin.gpioBank - MMAP_OFFSET + GPIO_DATAOUT) / GPIO_REGISTER_SIZE] &= ~ (1 << pin.bankId);
        break;

      case HIGH:
        // Set HIGH value to this pin.
        mapAddress[(pin.gpioBank - MMAP_OFFSET + GPIO_DATAOUT) / GPIO_REGISTER_SIZE] |= (1 << pin.bankId);
        break;

      default:
        std::cout << "Undefined pinValue: " << value << "!" << std::endl;
        exit (EXIT_FAILURE);
    }
  }

  void memGPIO::digitalWrite (std::string pinNumber, std::string pinValue) {
    // Get the gpioPin from pin key.
    gpioPin pin = getPin (pinNumber);
    // Get the pin value from string.
    const char value = getPinValue (pinValue);

    // Set pinValue value to this pinNumber.
    digitalWrite (pin, value);
  }

  const memGPIO::gpioPin memGPIO::getPin (std::string pinNumber) {
    // Search pinNumber in map.
    if (PIN_INDEX.count (pinNumber) > 0) {
      return PIN_INDEX.at (pinNumber);
    }
    else {
      std::cout << "Undefined pin: " << pinNumber << "!" << std::endl;
      exit (EXIT_FAILURE);
    }
  }

  void memGPIO::pinMode (const memGPIO::gpioPin pin, const unsigned char direction, unsigned char mux, unsigned char pullup, unsigned char slew) {
    std::string pinTemplate = "bspm";

    if (pin.name.compare (0, 3, "AIN") == 0) {
      std::cout << "Analog PINs is not yet supported!!" << std::endl;
      exit (EXIT_FAILURE);
    }

    switch (direction) {
      case OUTPUT:
        if (pin.led.compare (0, 3, "usr") == 0 && mux != 7) {
          std::cout << "GPIO mux value only supports '7' for LEDs!" << std::endl;
          exit (EXIT_FAILURE);
        }
        // Register the pin as OUTPUT.
        mapAddress[(pin.gpioBank - MMAP_OFFSET + GPIO_OE) / GPIO_REGISTER_SIZE] &= ~ (1 << pin.bankId);
        break;

      case INPUT:
        if (pin.led.compare (0, 3, "usr") == 0) {
          std::cout << "pinMode only supports GPIO output for LEDs!" << std::endl;
          exit (EXIT_FAILURE);
        }
        // Register the pin as INPUT.
        mapAddress[(pin.gpioBank - MMAP_OFFSET + GPIO_OE) / GPIO_REGISTER_SIZE] |= (1 << pin.bankId);
        break;

      default:
        std::cout << "Undefined direction " << direction << "!" << std::endl;
        exit (EXIT_FAILURE);
    }

    if (direction == INPUT_PULLUP) {
     pullup = PULLUP;
    }

    pullup = pullup || ((direction == INPUT) ? PULLDOWN : DISABLED);

    // Default to GPIO mode
    if (!(mux >= 1 && mux <= 7)) {
      mux = 7;
    }

    if (mux == pin.pwm.muxMode) {
      pinTemplate = "bspwm";
    }


    if (pin.led.compare (0, 3, "usr") == 0) {
      // Set LED trigger to "gpio".
      setLEDPinToGPIO (pin);
    }
    else {
      // Figure out pin data.
      int pinData = figureOutPinData (direction, mux, pullup, slew);

      // Tells a driver to configure the pin multiplexers as desired.
      setPinMode (pin, pinData, pinTemplate);
    }
  }

  void memGPIO::pinMode (std::string pinNumber, std::string pinDirection, unsigned char pinMux, std::string pinPullup, std::string pinSlew) {
    // Get the gpioPin from pin key.
    gpioPin pin = getPin (pinNumber);
    // Get the pin direction from string.
    const unsigned char direction = getPinDirection (pinDirection);
    // Get the pull mode from string.
    unsigned char pullup = getPullMode (pinPullup);
    // Get the slew value from string.
    unsigned char slew = getSlewValue (pinSlew);

    // Set the pin mode.
    pinMode (pin, direction, pinMux, pullup, slew);
  }

  void memGPIO::resetLEDPin0ToDefault () {
    // Get the gpioPin from pin key.
    gpioPin usr0 = getPin ("USR0");

    // Set LED pin trigger to default.
    setLEDPinToDefault (usr0);
  }

  void memGPIO::resetLEDPin1ToDefault () {
    // Get the gpioPin from pin key.
    gpioPin usr1 = getPin ("USR1");

    // Set LED pin trigger to default.
    setLEDPinToDefault (usr1);
  }

  void memGPIO::resetLEDPin2ToDefault () {
    // Get the gpioPin from pin key.
    gpioPin usr2 = getPin ("USR2");

    // Set LED pin trigger to default.
    setLEDPinToDefault (usr2);
  }

  void memGPIO::resetLEDPin3ToDefault () {
    // Get the gpioPin from pin key.
    gpioPin usr3 = getPin ("USR3");

    // Set LED pin trigger to default.
    setLEDPinToDefault (usr3);
  }

  void memGPIO::resetLEDPinsToDefault () {
    // Set all LED pins trigger to default.
    resetLEDPin0ToDefault ();
    resetLEDPin1ToDefault ();
    resetLEDPin2ToDefault ();
    resetLEDPin3ToDefault ();
  }

  /**
   * Private methods.
   */

  void memGPIO::checkKernelVersion () {
    struct utsname uName;

    if(uname (&uName) == -1) {
      perror ("Unable to determine the current Linux Kernel version");
      exit (EXIT_FAILURE);
    }
    else {
      int major, minor;

      // Get kernel version.
      sscanf (uName.release, "%d.%d.", &major, &minor);
      if (major >= 3 && minor >= 8) {
        oldKernel = false;
      }
      else {
        oldKernel = true;
      }
    }
  }

  std::string memGPIO::composeDTS (const memGPIO::gpioPin pin, int pinData, std::string pinTemplate) {
    std::string dtsTemplate, pinDotKey, mode, hexOffset, hexData;

    // Get the correct template.
    if (pinTemplate == "bspm") {
      dtsTemplate = BSPM_TEMPLATE;
    }
    else {
      dtsTemplate = BSPWM_TEMPLATE;

      // Replace all exclusive PWM pin patterns.
      easyUtils::str_replace_all (dtsTemplate, "!PWM_MODULE!", pin.pwm.module);
      easyUtils::str_replace_all (dtsTemplate, "!PWM_INDEX!", std::to_string (pin.pwm.index));
      easyUtils::str_replace_all (dtsTemplate, "!DUTY_CYCLE!", "500000");
    }

    pinDotKey = pin.key;
    easyUtils::str_replace (pinDotKey, "_", ".");

    mode = getPinMode (pin, pinData&7);
    // Convert some values to hexadecimal.
    hexOffset += "0x" + easyUtils::intToHex (pin.muxRegOffset);
    hexData += "0x" + easyUtils::intToHex (pinData);

    // Replace all patterns.
    easyUtils::str_replace_all (dtsTemplate, "!PIN_KEY!", pin.key);
    easyUtils::str_replace_all (dtsTemplate, "!PIN_DOT_KEY!", pinDotKey);
    easyUtils::str_replace_all (dtsTemplate, "!PIN_FUNCTION!", mode);
    easyUtils::str_replace_all (dtsTemplate, "!PIN_OFFSET!", hexOffset);
    easyUtils::str_replace_all (dtsTemplate, "!DATA!", hexData);

    return dtsTemplate;

  }

  void memGPIO::createDTBO (std::string dtboFilename, std::string dtsFilename) {
    int ret;
    std::string command;
    command += "dtc -O dtb -o " + dtboFilename + " -b 0 -@ " + dtsFilename;

    // Compile to a binary format DTB (flattened device tree format).
    if((ret = system (command.c_str ()))) {
      std::cout << "Unable to create DTBO file: " << dtboFilename << "!" << std::endl;
      exit (EXIT_FAILURE);
    }
  }

  void memGPIO::createDTS (const memGPIO::gpioPin pin, int pinData, std::string dtsFilename, std::string pinTemplate) {
    // Compose the driver for the pin.
    std::string dtsFile (composeDTS (pin, pinData, pinTemplate));

    // Save the driver for the pin.
    easyUtils::writeTextFile (dtsFilename, dtsFile);
  }

  void memGPIO::enableClockModules () {
    // Enable disabled GPIO module clocks.
    if (mapAddress[(CM_WKUP_GPIO0_CLKCTRL - MMAP_OFFSET) / GPIO_REGISTER_SIZE] & IDLEST_MASK) {
      mapAddress[(CM_WKUP_GPIO0_CLKCTRL - MMAP_OFFSET) / GPIO_REGISTER_SIZE] |= MODULEMODE_ENABLE;
      // Wait for the enable complete.
      while (mapAddress[(CM_WKUP_GPIO0_CLKCTRL - MMAP_OFFSET) / GPIO_REGISTER_SIZE] & IDLEST_MASK);
    }
    if (mapAddress[(CM_PER_GPIO1_CLKCTRL - MMAP_OFFSET) / GPIO_REGISTER_SIZE] & IDLEST_MASK) {
      mapAddress[(CM_PER_GPIO1_CLKCTRL - MMAP_OFFSET) / GPIO_REGISTER_SIZE] |= MODULEMODE_ENABLE;
      // Wait for the enable complete.
      while (mapAddress[(CM_PER_GPIO1_CLKCTRL - MMAP_OFFSET) / GPIO_REGISTER_SIZE] & IDLEST_MASK);
    }
    if (mapAddress[(CM_PER_GPIO2_CLKCTRL - MMAP_OFFSET) / GPIO_REGISTER_SIZE] & IDLEST_MASK) {
      mapAddress[(CM_PER_GPIO2_CLKCTRL - MMAP_OFFSET) / GPIO_REGISTER_SIZE] |= MODULEMODE_ENABLE;
      // Wait for the enable complete.
      while (mapAddress[(CM_PER_GPIO2_CLKCTRL - MMAP_OFFSET) / GPIO_REGISTER_SIZE] & IDLEST_MASK);
    }
    if (mapAddress[(CM_PER_GPIO3_CLKCTRL - MMAP_OFFSET) / GPIO_REGISTER_SIZE] & IDLEST_MASK) {
      mapAddress[(CM_PER_GPIO3_CLKCTRL - MMAP_OFFSET) / GPIO_REGISTER_SIZE] |= MODULEMODE_ENABLE;
      // Wait for the enable complete.
      while (mapAddress[(CM_PER_GPIO3_CLKCTRL - MMAP_OFFSET) / GPIO_REGISTER_SIZE] & IDLEST_MASK);
    }

/*    // Enable disabled EHRPWM module clocks.
    if (mapAddress[(CM_PER_EPWMSS0_CLKCTRL - MMAP_OFFSET) / GPIO_REGISTER_SIZE] & IDLEST_MASK) {
      mapAddress[(CM_PER_EPWMSS0_CLKCTRL - MMAP_OFFSET) / GPIO_REGISTER_SIZE] |= MODULEMODE_ENABLE;
      // Wait for the enable complete.
      while (mapAddress[(CM_PER_EPWMSS0_CLKCTRL - MMAP_OFFSET) / GPIO_REGISTER_SIZE] & IDLEST_MASK);
    }
    if (mapAddress[(CM_PER_EPWMSS1_CLKCTRL - MMAP_OFFSET) / GPIO_REGISTER_SIZE] & IDLEST_MASK) {
      mapAddress[(CM_PER_EPWMSS1_CLKCTRL - MMAP_OFFSET) / GPIO_REGISTER_SIZE] |= MODULEMODE_ENABLE;
      // Wait for the enable complete.
      while (mapAddress[(CM_PER_EPWMSS1_CLKCTRL - MMAP_OFFSET) / GPIO_REGISTER_SIZE] & IDLEST_MASK);
    }
    if (mapAddress[(CM_PER_EPWMSS2_CLKCTRL - MMAP_OFFSET) / GPIO_REGISTER_SIZE] & IDLEST_MASK) {
      mapAddress[(CM_PER_EPWMSS2_CLKCTRL - MMAP_OFFSET) / GPIO_REGISTER_SIZE] |= MODULEMODE_ENABLE;
      // Wait for the enable complete.
      while (mapAddress[(CM_PER_EPWMSS2_CLKCTRL - MMAP_OFFSET) / GPIO_REGISTER_SIZE] & IDLEST_MASK);
    }

    // Enable ADC module clock, if disabled.
    if (mapAddress[(CM_WKUP_ADC_TSC_CLKCTRL - MMAP_OFFSET) / GPIO_REGISTER_SIZE] & IDLEST_MASK) {
      mapAddress[(CM_WKUP_ADC_TSC_CLKCTRL - MMAP_OFFSET) / GPIO_REGISTER_SIZE] |= MODULEMODE_ENABLE;
      // Wait for the enable complete.
      while (mapAddress[(CM_WKUP_ADC_TSC_CLKCTRL - MMAP_OFFSET) / GPIO_REGISTER_SIZE] & IDLEST_MASK);
    }

    // Software reset.
    mapAddress[(ADC_SYSCONFIG - MMAP_OFFSET) / GPIO_REGISTER_SIZE] |= ADC_SOFTRESET;
    while (mapAddress[(ADC_SYSCONFIG - MMAP_OFFSET) / GPIO_REGISTER_SIZE] & ADC_SOFTRESET);

    // Make sure STEPCONFIG write protect is off.
    mapAddress[(ADC_CTRL - MMAP_OFFSET) / GPIO_REGISTER_SIZE] |= ADC_STEPCONFIG_WRITE_PROTECT_OFF;

    // Set STEPCONFIG1-STEPCONFIG8 to correspond to ADC inputs 0-7.
    mapAddress[(ADCSTEPCONFIG1 - MMAP_OFFSET) / GPIO_REGISTER_SIZE] |= ADCSTEPCONFIG1_VALUE;
    mapAddress[(ADCSTEPDELAY1 - MMAP_OFFSET) / GPIO_REGISTER_SIZE] |= ADCSTEPDELAY;

    mapAddress[(ADCSTEPCONFIG2 - MMAP_OFFSET) / GPIO_REGISTER_SIZE] |= ADCSTEPCONFIG2_VALUE;
    mapAddress[(ADCSTEPDELAY2 - MMAP_OFFSET) / GPIO_REGISTER_SIZE] |= ADCSTEPDELAY;

    mapAddress[(ADCSTEPCONFIG3 - MMAP_OFFSET) / GPIO_REGISTER_SIZE] |= ADCSTEPCONFIG3_VALUE;
    mapAddress[(ADCSTEPDELAY3 - MMAP_OFFSET) / GPIO_REGISTER_SIZE] |= ADCSTEPDELAY;

    mapAddress[(ADCSTEPCONFIG4 - MMAP_OFFSET) / GPIO_REGISTER_SIZE] |= ADCSTEPCONFIG4_VALUE;
    mapAddress[(ADCSTEPDELAY4 - MMAP_OFFSET) / GPIO_REGISTER_SIZE] |= ADCSTEPDELAY;

    mapAddress[(ADCSTEPCONFIG5 - MMAP_OFFSET) / GPIO_REGISTER_SIZE] |= ADCSTEPCONFIG5_VALUE;
    mapAddress[(ADCSTEPDELAY5 - MMAP_OFFSET) / GPIO_REGISTER_SIZE] |= ADCSTEPDELAY;

    mapAddress[(ADCSTEPCONFIG6 - MMAP_OFFSET) / GPIO_REGISTER_SIZE] |= ADCSTEPCONFIG6_VALUE;
    mapAddress[(ADCSTEPDELAY6 - MMAP_OFFSET) / GPIO_REGISTER_SIZE] |= ADCSTEPDELAY;

    mapAddress[(ADCSTEPCONFIG7 - MMAP_OFFSET) / GPIO_REGISTER_SIZE] |= ADCSTEPCONFIG7_VALUE;
    mapAddress[(ADCSTEPDELAY7 - MMAP_OFFSET) / GPIO_REGISTER_SIZE] |= ADCSTEPDELAY;

    mapAddress[(ADCSTEPCONFIG8 - MMAP_OFFSET) / GPIO_REGISTER_SIZE] |= ADCSTEPCONFIG8_VALUE;
    mapAddress[(ADCSTEPDELAY8 - MMAP_OFFSET) / GPIO_REGISTER_SIZE] |= ADCSTEPDELAY;

    // Enable ADC subsystem, leaving write protect off.
    mapAddress[(ADC_CTRL - MMAP_OFFSET) / GPIO_REGISTER_SIZE] |= TSC_ADC_SS_ENABLE;*/
  }

  int memGPIO::figureOutPinData (const unsigned char direction, unsigned char mux, unsigned char pullup, unsigned char slew) {
    int pinData = 0;

    if(slew == SLOW) {
      pinData |= 0x40;
    }

    if(direction != OUTPUT) {
      pinData |= 0x20;
    }

    switch(pullup) {
      case DISABLED:
        pinData |= 0x08;
        break;

      case PULLUP:
        pinData |= 0x10;
        break;

      default:
        break;
    }

    pinData |= (mux & 0x07);

    return(pinData);
  }

  const unsigned char memGPIO::getPinDirection (std::string pinDirection) {
    // Get the direction from string.
    if (easyUtils::str_toupper (pinDirection) == "OUTPUT") {
      return OUTPUT;
    }
    else if (easyUtils::str_toupper (pinDirection) == "INPUT") {
      return INPUT;
    }
    else if (easyUtils::str_toupper (pinDirection) == "INPUT_PULLUP") {
      return INPUT_PULLUP;
    }
    else {
      std::cout << "Undefined pin direction: " << pinDirection << "!" << std::endl;
      exit (EXIT_FAILURE);
    }
  }

  std::string memGPIO::getPinMode (const memGPIO::gpioPin pin, unsigned char pinMode) {
    // Get the pin mode from value.
    switch (pinMode) {
      case 0:
        return pin.modes.mode0;

      case 1:
        return pin.modes.mode1;

      case 2:
        return pin.modes.mode2;

      case 3:
        return pin.modes.mode3;

      case 4:
        return pin.modes.mode4;

      case 5:
        return pin.modes.mode5;

      case 6:
        return pin.modes.mode6;

      case 7:
        return pin.modes.mode7;

      default:
        std::cout << "Undefined pin mode: " << pinMode << "!" << std::endl;
        exit (EXIT_FAILURE);
    }
  }

  const unsigned char memGPIO::getPinValue (std::string pinValue) {
    // Get the pin value from string.
    if (easyUtils::str_toupper (pinValue) == "LOW") {
      return LOW;
    }
    else if (easyUtils::str_toupper (pinValue) == "HIGH") {
      return HIGH;
    }
    else {
      std::cout << "Undefined pin value: " << pinValue << "!" << std::endl;
      exit (EXIT_FAILURE);
    }
  }

  const unsigned char memGPIO::getPullMode (std::string pullMode) {
    // Get the pin pull mode from string.
    if (easyUtils::str_toupper (pullMode) == "PULLDOWN") {
      return PULLDOWN;
    }
    else if (easyUtils::str_toupper (pullMode) == "PULLUP") {
      return PULLUP;
    }
    else if (easyUtils::str_toupper (pullMode) == "DISABLED") {
      return DISABLED;
    }
    else {
      std::cout << "Undefined pull mode: " << pullMode << "!" << std::endl;
      exit (EXIT_FAILURE);
    }
  }

  const unsigned char memGPIO::getSlewValue (std::string slewValue) {
    // Get the pin slew value from string.
    if (easyUtils::str_toupper (slewValue) == "SLOW") {
      return SLOW;
    }
    else if (easyUtils::str_toupper (slewValue) == "FAST") {
      return FAST;
    }
    else {
      std::cout << "Undefined slew value: " << slewValue << "!" << std::endl;
      exit (EXIT_FAILURE);
    }
  }

  const std::string memGPIO::initializeBSPMTemplate () {
    return (std::string) "/**\
* Copyright (C) 2013 CircuitCo\
* Copyright (C) 2013 Texas Instruments\
*\
* This program is free software; you can redistribute it and/or modify\
* it under the terms of the GNU General Public License version 2 as\
* published by the Free Software Foundation.\
*\
* This is a template-generated file from memGPIO\
*/\
\
/dts-v1/;\
/plugin/;\
\
/{\
  compatible = \"ti,beaglebone\", \"ti,beaglebone-black\";\
  part_number = \"BS_PINMODE_!PIN_KEY!_!DATA!\";\
\
  exclusive-use =\
    \"!PIN_DOT_KEY!\",\
    \"!PIN_FUNCTION!\";\
\
  fragment@0 {\
    target = <&am33xx_pinmux>;\
    __overlay__ {\
      bs_pinmode_!PIN_KEY!_!DATA!: pinmux_bs_pinmode_!PIN_KEY!_!DATA! {\
        pinctrl-single,pins = <!PIN_OFFSET! !DATA!>;\
      };\
    };\
  };\
\
  fragment@1 {\
    target = <&ocp>;\
    __overlay__ {\
      bs_pinmode_!PIN_KEY!_!DATA!_pinmux {\
        compatible = \"bone-pinmux-helper\";\
        status = \"okay\";\
        pinctrl-names = \"default\";\
        pinctrl-0 = <&bs_pinmode_!PIN_KEY!_!DATA!>;\
      };\
    };\
  };\
};\
";
  }

  const std::string memGPIO::initializeBSPWMTemplate () {
    return (std::string) "/**\
* Copyright (C) 2013 CircuitCo\
* Copyright (C) 2013 Texas Instruments\
*\
*  program is free software; you can redistribute it and/or modify\
* it under the terms of the GNU General Public License version 2 as\
* published by the Free Software Foundation.\
*\
* This is a template-generated file from memGPIO\
*/\
\
/dts-v1/;\
/plugin/;\
\
/{\
  compatible = \"ti,beaglebone\", \"ti,beaglebone-black\";\
\
  /* identification */\
  part-number = \"BS_PWM_!PIN_KEY!_!DATA!\";\
\
  /* state the resources this cape uses */\
  exclusive-use =\
    /* the pin header uses */\
    \"!PIN_DOT_KEY!\",\
    /* the hardware IP uses */\
    \"!PIN_FUNCTION!\";\
\
  fragment@0 {\
    target = <&am33xx_pinmux>;\
    __overlay__ {\
      bs_pwm_!PIN_KEY!_!DATA!: pinmux_bs_pwm_!PIN_KEY!_!DATA! {\
        pinctrl-single,pins = <!PIN_OFFSET! !DATA!>;\
      };\
    };\
  };\
\
  fragment@1 {\
    target = <&ocp>;\
    __overlay__ {\
      bs_pwm_test_!PIN_KEY! {\
        compatible    = \"pwm_test\";\
        pwms          = <&!PWM_MODULE! !PWM_INDEX! !DUTY_CYCLE! 1>;\
        pwm-names     = \"PWM_!PIN_KEY!\";\
\
        pinctrl-names = \"default\";\
        pinctrl-0     = <&bs_pwm_!PIN_KEY!_!DATA!>;\
\
        enabled       = <1>;\
        duty          = <0>;\
        status        = \"okay\";\
      };\
    };\
  };\
};\
";
  }

  std::map<std::string, const memGPIO::gpioPin> memGPIO::initializePins () {
    std::map<std::string, const gpioPin> pins;

    // Initialize all pin values.
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "USR0", {
          "GPIO1_21",
          "USR0",
          "usr0",
          GPIO1,
          53,
          21,
          0,
          "gpmc_a5",
          0x054,
          {},
          {},
          {},
          (gpioModes) {
            "gpmc_a5",
            "gmii2_txd0",
            "rgmii2_td0",
            "rmii2_txd0",
            "gpmc_a21",
            "pr1_mii1_rxd3",
            "eqep1b_in",
            "gpio1_21"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "USR1", {
          "GPIO1_22",
          "USR1",
          "usr1",
          GPIO1,
          54,
          22,
          0,
          "gpmc_a6",
          0x058,
          {},
          {},
          {},
          (gpioModes) {
            "gpmc_a6",
            "gmii2_txclk",
            "rgmii2_tclk",
            "mmc2_dat4",
            "gpmc_a22",
            "pr1_mii1_rxd2",
            "eqep1_index",
            "gpio1_22"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "USR2", {
          "GPIO1_23",
          "USR2",
          "usr2",
          GPIO1,
          55,
          23,
          0,
          "gpmc_a7",
          0x05C,
          {},
          {},
          {},
          (gpioModes) {
            "gpmc_a7",
            "gmii2_rxclk",
            "rgmii2_rclk",
            "mmc2_dat5",
            "gpmc_a23",
            "pr1_mii1_rxd1",
            "eqep1_strobe",
            "gpio1_23"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "USR3", {
          "GPIO1_24",
          "USR3",
          "usr3",
          GPIO1,
          56,
          24,
          0,
          "gpmc_a8",
          0x060,
          {},
          {},
          {},
          (gpioModes) {
            "gpmc_a8",
            "gmii2_rxd3",
            "rgmii2_rd3",
            "mmc2_dat6",
            "gpmc_a24",
            "pr1_mii1_rxd0",
            "mcasp0_aclkx",
            "gpio1_24"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P8_3", {
          "GPIO1_6",
          "P8_3",
          "",
          GPIO1,
          38,
          6,
          26,
          "gpmc_ad6",
          0x018,
          {},
          {},
          {},
          (gpioModes) {
            "gpmc_ad6",
            "mmc1_dat6",
            "N/A",
            "N/A",
            "N/A",
            "N/A",
            "N/A",
            "gpio1_6"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P8_4", {
          "GPIO1_7",
          "P8_4",
          "",
          GPIO1,
          39,
          7,
          27,
          "gpmc_ad7",
          0x01C,
          {},
          {},
          {},
          (gpioModes) {
            "gpmc_ad7",
            "mmc1_dat7",
            "N/A",
            "N/A",
            "N/A",
            "N/A",
            "N/A",
            "gpio1_7"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P8_5", {
          "GPIO1_2",
          "P8_5",
          "",
          GPIO1,
          34,
          2,
          22,
          "gpmc_ad2",
          0x008,
          {},
          {},
          {},
          (gpioModes) {
            "gpmc_ad2",
            "mmc1_dat2",
            "N/A",
            "N/A",
            "N/A",
            "N/A",
            "N/A",
            "gpio1_2"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P8_6", {
          "GPIO1_3",
          "P8_6",
          "",
          GPIO1,
          35,
          3,
          23,
          "gpmc_ad3",
          0x00C,
          {},
          {},
          {},
          (gpioModes) {
            "gpmc_ad3",
            "mmc1_dat3",
            "N/A",
            "N/A",
            "N/A",
            "N/A",
            "N/A",
            "gpio1_3"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P8_7", {
          "TIMER4",
          "P8_7",
          "",
          GPIO2,
          66,
          2,
          41,
          "gpmc_advn_ale",
          0x090,
          {},
          {},
          {},
          (gpioModes) {
            "gpmc_advn_ale",
            "N/A",
            "N/A",
            "N/A",
            "N/A",
            "N/A",
            "N/A",
            "mmc1_sdcd"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P8_8", {
          "TIMER7",
          "P8_8",
          "",
          GPIO2,
          67,
          3,
          44,
          "gpmc_oen_ren",
          0x094,
          {},
          {},
          {},
          (gpioModes) {
            "gpmc_oen_ren",
            "N/A",
            "N/A",
            "N/A",
            "N/A",
            "N/A",
            "N/A",
            "gpio2_3"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P8_9", {
          "TIMER5",
          "P8_9",
          "",
          GPIO2,
          69,
          5,
          42,
          "gpmc_ben0_cle",
          0x09C,
          {},
          {},
          {},
          (gpioModes) {
            "gpmc_ben0_cle",
            "N/A",
            "N/A",
            "N/A",
            "N/A",
            "N/A",
            "N/A",
            "gpio2_5"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P8_10", {
          "TIMER6",
          "P8_10",
          "",
          GPIO2,
          68,
          4,
          43,
          "gpmc_wen",
          0x098,
          {},
          {},
          {},
          (gpioModes) {
            "gpmc_wen",
            "N/A",
            "N/A",
            "N/A",
            "N/A",
            "N/A",
            "N/A",
            "gpio2_4"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P8_11", {
          "GPIO1_13",
          "P8_11",
          "",
          GPIO1,
          45,
          13,
          29,
          "gpmc_ad13",
          0x034,
          {},
          {},
          {},
          (gpioModes) {
            "gpmc_ad13",
            "lcd_data18",
            "mmc1_dat5",
            "mmc2_dat1",
            "eqep2B_in",
            "pr1_mii0_txd",
            "pr1_pru0_pru_r30_15",
            "gpio1_13"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P8_12", {
          "GPIO1_12",
          "P8_12",
          "",
          GPIO1,
          44,
          12,
          28,
          "gpmc_ad12",
          0x030,
          {},
          {},
          {},
          (gpioModes) {
            "gpmc_ad12",
            "lcd_data19",
            "mmc1_dat4",
            "mmc2_dat0",
            "eqep2a_in",
            "pr1_mii0_txd2",
            "pr1_pru0_pru_r30_14",
            "gpio1_12"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P8_13", {
          "EHRPWM2B",
          "P8_13",
          "",
          GPIO0,
          23,
          23,
          15,
          "gpmc_ad9",
          0x024,
          {},
          {},
          (gpioPwm) {
            "ehrpwm2",
            6,
            1,
            4,
            "ehrpwm.2:1",
            "EHRPWM2B"
          },
          (gpioModes) {
            "gpmc_ad9",
            "lcd_data22",
            "mmc1_dat1",
            "mmc2_dat5",
            "ehrpwm2B",
            "pr1_mii0_col",
            "N/A",
            "gpio0_23"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P8_14", {
          "GPIO0_26",
          "P8_14",
          "",
          GPIO0,
          26,
          26,
          16,
          "gpmc_ad10",
          0x028,
          {},
          {},
          {},
          (gpioModes) {
            "gpmc_ad10",
            "lcd_data21",
            "mmc1_dat2",
            "mmc2_dat6",
            "ehrpwm2_tripzone_input",
            "pr1_mii0_txen",
            "N/A",
            "gpio0_26"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P8_15", {
          "GPIO1_15",
          "P8_15",
          "",
          GPIO1,
          47,
          15,
          31,
          "gpmc_ad15",
          0x03C,
          {},
          {},
          {},
          (gpioModes) {
            "gpmc_ad15",
            "lcd_data16",
            "mmc1_dat7",
            "mmc2_dat3",
            "eqep2_strobe",
            "pr1_ecap0_ecap_capin_apwm_o",
            "pr1_pru0_pru_r31_15",
            "gpio1_15"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P8_16", {
          "GPIO1_14",
          "P8_16",
          "",
          GPIO1,
          46,
          14,
          30,
          "gpmc_ad14",
          0x038,
          {},
          {},
          {},
          (gpioModes) {
            "gpmc_ad14",
            "lcd_data17",
            "mmc1_dat6",
            "mmc2_dat2",
            "eqep2_index",
            "pr1_mii0_txd0",
            "pr1_pru0_pru_r31_14",
            "gpio1_14"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P8_17", {
          "GPIO0_27",
          "P8_17",
          "",
          GPIO0,
          27,
          27,
          17,
          "gpmc_ad11",
          0x02C,
          {},
          {},
          {},
          (gpioModes) {
            "gpmc_ad11",
            "lcd_data20",
            "mmc1_dat3",
            "mmc2_dat7",
            "ehrpwm0_synco",
            "pr1_mii0_txd3",
            "N/A",
            "gpio0_27"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P8_18", {
          "GPIO2_1",
          "P8_18",
          "",
          GPIO2,
          65,
          1,
          40,
          "gpmc_clk",
          0x08C,
          {},
          {},
          {},
          (gpioModes) {
            "gpmc_clk",
            "lcd_memory_clk_mux",
            "N/A",
            "mmc2_clk",
            "N/A",
            "N/A",
            "mcasp0_fsr",
            "gpio2_1"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P8_19", {
          "EHRPWM2A",
          "P8_19",
          "",
          GPIO0,
          22,
          22,
          14,
          "gpmc_ad8",
          0x020,
          {},
          {},
          (gpioPwm) {
            "ehrpwm2",
            5,
            0,
            4,
            "ehrpwm.2:0",
            "EHRPWM2A"
          },
          (gpioModes) {
            "gpmc_ad8",
            "lcd_data23",
            "mmc1_dat0",
            "mmc2_dat4",
            "ehrpwm2A",
            "pr1_mii_mt0_clk",
            "N/A",
            "gpio0_22"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P8_20", {
          "GPIO1_31",
          "P8_20",
          "",
          GPIO1,
          63,
          31,
          39,
          "gpmc_csn2",
          0x084,
          {},
          {},
          {},
          (gpioModes) {
            "gpmc_csn2",
            "gpmc_be1n",
            "mmc1_cmd",
            "pr1_edio_data_in7",
            "pr1_edio_data_out7",
            "pr1_pru1_pru_r30_13",
            "pr1_pru1_pru_r31_13",
            "gpio1_31"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P8_21", {
          "GPIO1_30",
          "P8_21",
          "",
          GPIO1,
          62,
          30,
          38,
          "gpmc_csn1",
          0x080,
          {},
          {},
          {},
          (gpioModes) {
            "gpmc_csn1",
            "gpmc_clk",
            "mmc1_clk",
            "pr1_edio_data_in6",
            "pr1_edio_data_out6",
            "pr1_pru1_pru_r30_12",
            "pr1_pru1_pru_r31_12",
            "gpio1_30"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P8_22", {
          "GPIO1_5",
          "P8_22",
          "",
          GPIO1,
          37,
          5,
          25,
          "gpmc_ad5",
          0x014,
          {},
          {},
          {},
          (gpioModes) {
            "gpmc_ad5",
            "mmc1_dat5",
            "N/A",
            "N/A",
            "N/A",
            "N/A",
            "N/A",
            "gpio1_5"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P8_23", {
          "GPIO1_4",
          "P8_23",
          "",
          GPIO1,
          36,
          4,
          24,
          "gpmc_ad4",
          0x010,
          {},
          {},
          {},
          (gpioModes) {
            "gpmc_ad4",
            "mmc1_dat4",
            "N/A",
            "N/A",
            "N/A",
            "N/A",
            "N/A",
            "gpio1_4"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P8_24", {
          "GPIO1_1",
          "P8_24",
          "",
          GPIO1,
          33,
          1,
          21,
          "gpmc_ad1",
          0x004,
          {},
          {},
          {},
          (gpioModes) {
            "gpmc_ad1",
            "mmc1_dat1",
            "N/A",
            "N/A",
            "N/A",
            "N/A",
            "N/A",
            "gpio1_1"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P8_25", {
          "GPIO1_0",
          "P8_25",
          "",
          GPIO1,
          32,
          0,
          20,
          "gpmc_ad0",
          0x000,
          {},
          {},
          {},
          (gpioModes) {
            "gpmc_ad0",
            "mmc1_dat0",
            "N/A",
            "N/A",
            "N/A",
            "N/A",
            "N/A",
            "gpio1_0"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P8_26", {
          "GPIO1_29",
          "P8_26",
          "",
          GPIO1,
          61,
          29,
          37,
          "gpmc_csn0",
          0x07C,
          {},
          {},
          {},
          (gpioModes) {
            "gpmc_csn0",
            "N/A",
            "N/A",
            "N/A",
            "N/A",
            "N/A",
            "N/A",
            "gpio1_29"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P8_27", {
          "GPIO2_22",
          "P8_27",
          "",
          GPIO2,
          86,
          22,
          57,
          "lcd_vsync",
          0x0e0,
          {},
          {},
          {},
          (gpioModes) {
            "lcd_vsync",
            "gpmc_a8",
            "N/A",
            "pr1_edio_data_in2",
            "pr1_edio_data_out2",
            "pr1_pru1_pru_r30_8",
            "pr1_pru1_pru_r31_8",
            "gpio2_22"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P8_28", {
          "GPIO2_24",
          "P8_28",
          "",
          GPIO2,
          88,
          24,
          59,
          "lcd_pclk",
          0x0e8,
          {},
          {},
          {},
          (gpioModes) {
            "lcd_pclk",
            "gpmc_a10",
            "pr1_mii0_crs",
            "pr1_edio_data_in4",
            "pr1_edio_data_out4",
            "pr1_pru1_pru_r30_10",
            "pr1_pru1_pru_r31_10",
            "gpio2_24"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P8_29", {
          "GPIO2_23",
          "P8_29",
          "",
          GPIO2,
          87,
          23,
          58,
          "lcd_hsync",
          0x0e4,
          {},
          {},
          {},
          (gpioModes) {
            "lcd_hsync",
            "gpmc_a9",
            "N/A",
            "pr1_edio_data_in3",
            "pr1_edio_data_out3",
            "pr1_pru1_pru_r30_9",
            "pr1_pru1_pru_r31_9",
            "gpio2_23"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P8_30", {
          "GPIO2_25",
          "P8_30",
          "",
          GPIO2,
          89,
          25,
          60,
          "lcd_ac_bias_en",
          0x0EC,
          {},
          {},
          {},
          (gpioModes) {
            "lcd_ac_bias_en",
            "gpmc_a11",
            "pr1_mii1_crs",
            "pr1_edio_data_in5",
            "pr1_edio_data_out5",
            "pr1_pru1_pru_r30_11",
            "pr1_pru1_pru_r31_11",
            "gpio2_25"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P8_31", {
          "UART5_CTSN",
          "P8_31",
          "",
          GPIO0,
          10,
          10,
          7,
          "lcd_data14",
          0x0d8,
          {},
          {},
          {},
          (gpioModes) {
            "lcd_data14",
            "gpmc_a18",
            "N/A",
            "mcasp0_axr1",
            "N/A",
            "N/A",
            "N/A",
            "gpio0_10"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P8_32", {
          "UART5_RTSN",
          "P8_32",
          "",
          GPIO0,
          11,
          11,
          8,
          "lcd_data15",
          0x0DC,
          {},
          {},
          {},
          (gpioModes) {
            "lcd_data15",
            "gpmc_a19",
            "N/A",
            "mcasp0_ahclkx",
            "mcasp0_axr3",
            "N/A",
            "N/A",
            "gpio0_11"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P8_33", {
          "UART4_RTSN",
          "P8_33",
          "",
          GPIO0,
          9,
          9,
          6,
          "lcd_data13",
          0x0d4,
          {},
          {},
          {},
          (gpioModes) {
            "lcd_data13",
            "gpmc_a17",
            "N/A",
            "mcasp0_fsr",
            "mcasp0_axr3",
            "N/A",
            "N/A",
            "gpio0_9"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P8_34", {
          "UART3_RTSN",
          "P8_34",
          "",
          GPIO2,
          81,
          17,
          56,
          "lcd_data11",
          0x0CC,
          {},
          {},
          (gpioPwm) {
            "ehrpwm1",
            4,
            1,
            2,
            "ehrpwm.1:1",
            "EHRPWM1B"
          },
          (gpioModes) {
            "lcd_data11",
            "gpmc_a15",
            "N/A",
            "mcasp0_ahclkr",
            "mcasp0_axr2",
            "N/A",
            "N/A",
            "gpio2_17"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P8_35", {
          "UART4_CTSN",
          "P8_35",
          "",
          GPIO0,
          8,
          8,
          5,
          "lcd_data12",
          0x0d0,
          {},
          {},
          {},
          (gpioModes) {
            "lcd_data12",
            "gpmc_a16",
            "N/A",
            "mcasp0_aclkr",
            "mcasp0_axr2",
            "N/A",
            "N/A",
            "gpio0_8"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P8_36", {
          "UART3_CTSN",
          "P8_36",
          "",
          GPIO2,
          80,
          16,
          55,
          "lcd_data10",
          0x0c8,
          {},
          {},
          (gpioPwm) {
            "ehrpwm1",
            3,
            0,
            2,
            "ehrpwm.1:0",
            "EHRPWM1A"
          },
          (gpioModes) {
            "lcd_data10",
            "gpmc_a14",
            "ehrpwm1A",
            "mcasp0_axr0",
            "mcasp0_axr0",
            "pr1_mii0_rxd1",
            "uart3_ctsn",
            "gpio2_16"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P8_37", {
          "UART5_TXD",
          "P8_37",
          "",
          GPIO2,
          78,
          14,
          53,
          "lcd_data8",
          0x0c0,
          {},
          {},
          {},
          (gpioModes) {
            "lcd_data8",
            "gpmc_a12",
            "N/A",
            "mcasp0_aclkx",
            "N/A",
            "N/A",
            "uart2_ctsn",
            "gpio2_14"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P8_38", {
          "UART5_RXD",
          "P8_38",
          "",
          GPIO2,
          79,
          15,
          54,
          "lcd_data9",
          0x0c4,
          {},
          {},
          {},
          (gpioModes) {
            "lcd_data9",
            "gpmc_a13",
            "N/A",
            "mcasp0_fsx",
            "N/A",
            "N/A",
            "uart2_rtsn",
            "gpio2_15"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P8_39", {
          "GPIO2_12",
          "P8_39",
          "",
          GPIO2,
          76,
          12,
          51,
          "lcd_data6",
          0x0b8,
          {},
          {},
          {},
          (gpioModes) {
            "lcd_data6",
            "gpmc_a6",
            "pr1_edio_data_in6",
            "eqep2_index",
            "pr1_edio_data_out6",
            "pr1_pru1_pru_r30_6",
            "pr1_pru1_pru_r31_6",
            "gpio2_12"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P8_40", {
          "GPIO2_13",
          "P8_40",
          "",
          GPIO2,
          77,
          13,
          52,
          "lcd_data7",
          0x0BC,
          {},
          {},
          {},
          (gpioModes) {
            "lcd_data7",
            "gpmc_a7",
            "pr1_edio_data_in7",
            "eqep2_strobe",
            "pr1_pru1_pru_r30_7",
            "pr1_pru_pru1_r30_7",
            "pr1_pru1_pru_r31_7",
            "gpio2_13"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P8_41", {
          "GPIO2_10",
          "P8_41",
          "",
          GPIO2,
          74,
          10,
          49,
          "lcd_data4",
          0x0b0,
          {},
          {},
          {},
          (gpioModes) {
            "lcd_data4",
            "gpmc_a4",
            "pr1_mii0_txd1",
            "eQEP2A_in",
            "N/A",
            "pr1_pru1_pru_r30_4",
            "pr1_pru1_pru_r31_4",
            "gpio2_10"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P8_42", {
          "GPIO2_11",
          "P8_42",
          "",
          GPIO2,
          75,
          11,
          50,
          "lcd_data5",
          0x0b4,
          {},
          {},
          {},
          (gpioModes) {
            "lcd_data5",
            "gpmc_a5",
            "pr1_mii0_txd0",
            "eqep2b_in",
            "N/A",
            "pr1_pru1_pru_r30_5",
            "pr1_pru1_pru_r31_5",
            "gpio2_11"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P8_43", {
          "GPIO2_8",
          "P8_43",
          "",
          GPIO2,
          72,
          8,
          47,
          "lcd_data2",
          0x0a8,
          {},
          {},
          {},
          (gpioModes) {
            "lcd_data2",
            "gpmc_a2",
            "pr1_mii0_txd3",
            "ehrpwm2_tripzone_input",
            "N/A",
            "pr1_pru1_pru_r30_2",
            "pr1_pru1_pru_r31_2",
            "gpio2_8"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P8_44", {
          "GPIO2_9",
          "P8_44",
          "",
          GPIO2,
          73,
          9,
          48,
          "lcd_data3",
          0x0AC,
          {},
          {},
          {},
          (gpioModes) {
            "lcd_data3",
            "gpmc_a3",
            "pr1_mii0_txd2",
            "ehrpwm0_synco",
            "N/A",
            "pr1_pru1_pru_r30_3",
            "pr1_pru1_pru_r31_3",
            "gpio2_9"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P8_45", {
          "GPIO2_6",
          "P8_45",
          "",
          GPIO2,
          70,
          6,
          45,
          "lcd_data0",
          0x0a0,
          {},
          {},
          (gpioPwm) {
            "ehrpwm2",
            5,
            0,
            3,
            "ehrpwm.2:0",
            "EHRPWM2A"
          },
          (gpioModes) {
            "lcd_data0",
            "gpmc_a0",
            "pr1_mii_mt0_clk",
            "ehrpwm2A",
            "N/A",
            "pr1_pru1_pru_r30_0",
            "pr1_pru1_pru_r31_0",
            "gpio2_6"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P8_46", {
          "GPIO2_7",
          "P8_46",
          "",
          GPIO2,
          71,
          7,
          46,
          "lcd_data1",
          0x0a4,
          {},
          {},
          (gpioPwm) {
            "ehrpwm2",
            6,
            1,
            3,
            "ehrpwm.2:1",
            "EHRPWM2B"
          },
          (gpioModes) {
            "lcd_data1",
            "gpmc_a1",
            "pr1_mii0_txen",
            "ehrpwm2B",
            "N/A",
            "pr1_pru1_pru_r30_1",
            "pr1_pru1_pru_r31_1",
            "gpio2_7"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P9_11", {
          "UART4_RXD",
          "P9_11",
          "",
          GPIO0,
          30,
          30,
          18,
          "gpmc_wait0",
          0x070,
          {},
          {},
          {},
          (gpioModes) {
            "gpmc_wait0",
            "mii2_crs",
            "N/A",
            "rmii2_crs_dv",
            "mmc1_sdcd",
            "N/A",
            "N/A",
            "gpio0_30"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P9_12", {
          "GPIO1_28",
          "P9_12",
          "",
          GPIO1,
          60,
          28,
          36,
          "gpmc_ben1",
          0x078,
          {},
          {},
          {},
          (gpioModes) {
            "gpmc_ben1",
            "mii2_col",
            "N/A",
            "mmc2_dat3",
            "N/A",
            "N/A",
            "mcasp0_aclkr",
            "gpio1_28"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P9_13", {
          "UART4_TXD",
          "P9_13",
          "",
          GPIO0,
          31,
          31,
          19,
          "gpmc_wpn",
          0x074,
          {},
          {},
          {},
          (gpioModes) {
            "gpmc_wpn",
            "mii2_rxerr",
            "N/A",
            "rmii2_rxerr",
            "mmc2_sdcd",
            "N/A",
            "N/A",
            "gpio0_31"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P9_14", {
          "EHRPWM1A",
          "P9_14",
          "",
          GPIO1,
          50,
          18,
          34,
          "gpmc_a2",
          0x048,
          {},
          {},
          (gpioPwm) {
            "ehrpwm1",
            3,
            0,
            6,
            "ehrpwm.1:0",
            "EHRPWM1A"
          },
          (gpioModes) {
            "gpmc_a2",
            "gmii2_txd3",
            "rgmii2_td3",
            "mmc2_dat1",
            "gpmc_a18",
            "pr1_mii1_txd2",
            "ehrpwm1A",
            "gpio1_18"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P9_15", {
          "GPIO1_16",
          "P9_15",
          "",
          GPIO1,
          48,
          16,
          32,
          "mii1_rxd3",
          0x040,
          {},
          {},
          {},
          (gpioModes) {
            "gpmc_a0",
            "gmii2_txen",
            "rgmii2_tctl",
            "rmii2_txen",
            "gpmc_a16",
            "pr1_mii_mt1_clk",
            "ehrpwm1_tripzone_input",
            "gpio1_16"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P9_16", {
          "EHRPWM1B",
          "P9_16",
          "",
          GPIO1,
          51,
          19,
          35,
          "gpmc_a3",
          0x04C,
          {},
          {},
          (gpioPwm) {
            "ehrpwm1",
            4,
            1,
            6,
            "ehrpwm.1:1",
            "EHRPWM1B"
          },
          (gpioModes) {
            "gpmc_a3",
            "gmii2_txd2",
            "rgmii2_td2",
            "mmc2_dat2",
            "gpmc_a19",
            "pr1_mii1_txd1",
            "ehrpwm1B",
            "gpio1_19"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P9_17", {
          "I2C1_SCL",
          "P9_17",
          "",
          GPIO0,
          5,
          5,
          3,
          "spi0_cs0",
          0x15C,
          {},
          {},
          {},
          (gpioModes) {
            "spi0_cs0",
            "mmc2_sdwp",
            "i2c1_scl",
            "N/A",
            "N/A",
            "N/A",
            "N/A",
            "gpio0_5"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P9_18", {
          "I2C1_SDA",
          "P9_18",
          "",
          GPIO0,
          4,
          4,
          2,
          "spi0_d1",
          0x158,
          {},
          {},
          {},
          (gpioModes) {
            "spi0_d1",
            "mmc1_sdwp",
            "i2c1_sda",
            "N/A",
            "N/A",
            "N/A",
            "N/A",
            "gpio0_4"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P9_19", {
          "I2C2_SCL",
          "P9_19",
          "",
          GPIO0,
          13,
          13,
          9,
          "uart1_rtsn",
          0x17C,
          {},
          {},
          {},
          (gpioModes) {
            "uart1_rtsn",
            "N/A",
            "d_can0_rx",
            "i2c2_scl",
            "spi1_cs1",
            "N/A",
            "N/A",
            "gpio0_13"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P9_20", {
          "I2C2_SDA",
          "P9_20",
          "",
          GPIO0,
          12,
          12,
          10,
          "uart1_ctsn",
          0x178,
          {},
          {},
          {},
          (gpioModes) {
            "uart1_ctsn",
            "N/A",
            "d_can0_tx",
            "i2c2_sda",
            "spi1_cs0",
            "N/A",
            "N/A",
            "gpio0_12"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P9_21", {
          "UART2_TXD",
          "P9_21",
          "",
          GPIO0,
          3,
          3,
          1,
          "spi0_d0",
          0x154,
          {},
          {},
          (gpioPwm) {
            "ehrpwm0",
            1,
            1,
            3,
            "ehrpwm.0:1",
            "EHRPWM0B"
          },
          (gpioModes) {
            "spi0_d0",
            "uart2_txd",
            "i2c2_scl",
            "N/A",
            "N/A",
            "N/A",
            "N/A",
            "gpio0_3"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P9_22", {
          "UART2_RXD",
          "P9_22",
          "",
          GPIO0,
          2,
          2,
          0,
          "spi0_sclk",
          0x150,
          {},
          {},
          (gpioPwm) {
            "ehrpwm0",
            0,
            0,
            3,
            "ehrpwm.0:0",
            "EHRPWM0A"
          },
          (gpioModes) {
            "spi0_sclk",
            "uart2_rxd",
            "i2c2_sda",
            "N/A",
            "N/A",
            "N/A",
            "N/A",
            "gpio0_2"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P9_23", {
          "GPIO1_17",
          "P9_23",
          "",
          GPIO1,
          49,
          17,
          33,
          "gpmc_a1",
          0x044,
          {},
          {},
          {},
          (gpioModes) {
            "gpmc_a1",
            "gmii2_rxdv",
            "rgmii2_rctl",
            "mmc2_dat0",
            "gpmc_a17",
            "pr1_mii1_txd3",
            "ehrpwm0_synco",
            "gpio1_17"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P9_24", {
          "UART1_TXD",
          "P9_24",
          "",
          GPIO0,
          15,
          15,
          12,
          "uart1_txd",
          0x184,
          {},
          {},
          {},
          (gpioModes) {
            "uart1_txd",
            "mmc2_sdwp",
            "d_can1_rx",
            "i2c1_scl",
            "N/A",
            "pr1_uart0_txd_mux1",
            "N/A",
            "gpio0_15"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P9_25", {
          "GPIO3_21",
          "P9_25",
          "",
          GPIO3,
          117,
          21,
          66,
          "mcasp0_ahclkx",
          0x1AC,
          {},
          {},
          {},
          (gpioModes) {
            "mcasp0_ahclkx",
            "N/A",
            "mcasp0_axr3",
            "mcasp1_axr1",
            "N/A",
            "N/A",
            "N/A",
            "gpio3_21"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P9_26", {
          "UART1_RXD",
          "P9_26",
          "",
          GPIO0,
          14,
          14,
          11,
          "uart1_rxd",
          0x180,
          {},
          {},
          {},
          (gpioModes) {
            "uart1_rxd",
            "mmc1_sdwp",
            "d_can1_tx",
            "i2c1_sda",
            "N/A",
            "pr1_uart0_rxd_mux1",
            "N/A",
            "gpio0_14"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P9_27", {
          "GPIO3_19",
          "P9_27",
          "",
          GPIO3,
          115,
          19,
          64,
          "mcasp0_fsr",
          0x1a4,
          {},
          {},
          {},
          (gpioModes) {
            "mcasp0_fsr",
            "N/A",
            "mcasp0_axr3",
            "mcasp1_fsx",
            "N/A",
            "pr1_pru0_pru_r30_5",
            "N/A",
            "gpio3_19"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P9_28", {
          "SPI1_CS0",
          "P9_28",
          "",
          GPIO3,
          113,
          17,
          63,
          "mcasp0_ahclkr",
          0x19C,
          {},
          {},
          (gpioPwm) {
            "ecap2",
            7,
            2,
            4,
            "ecap.2",
            "ECAPPWM2"
          },
          (gpioModes) {
            "mcasp0_ahclkr",
            "N/A",
            "mcasp0_axr2",
            "spi1_cs0",
            "eCAP2_in_PWM2_out",
            "N/A",
            "N/A",
            "gpio3_17"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P9_29", {
          "SPI1_D0",
          "P9_29",
          "",
          GPIO3,
          111,
          15,
          61,
          "mcasp0_fsx",
          0x194,
          {},
          {},
          (gpioPwm) {
            "ehrpwm0",
            1,
            1,
            1,
            "ehrpwm.0:1",
            "EHRPWM0B"
          },
          (gpioModes) {
            "mcasp0_fsx",
            "ehrpwm0B",
            "N/A",
            "spi1_d0",
            "mmc1_sdcd",
            "N/A",
            "N/A",
            "gpio3_15"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P9_30", {
          "SPI1_D1",
          "P9_30",
          "",
          GPIO3,
          112,
          16,
          62,
          "mcasp0_axr0",
          0x198,
          {},
          {},
          {},
          (gpioModes) {
            "mcasp0_axr0",
            "N/A",
            "N/A",
            "spi1_d1",
            "mmc2_sdcd",
            "N/A",
            "N/A",
            "gpio3_16"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P9_31", {
          "SPI1_SCLK",
          "P9_31",
          "",
          GPIO3,
          110,
          14,
          65,
          "mcasp0_aclkx",
          0x190,
          {},
          {},
          (gpioPwm) {
            "ehrpwm0",
            0,
            0,
            1,
            "ehrpwm.0:0",
            "EHRPWM0A"
          },
          (gpioModes) {
            "mcasp0_aclkx",
            "ehrpwm0A",
            "N/A",
            "spi1_sclk",
            "mmc0_sdcd",
            "N/A",
            "N/A",
            "gpio3_14"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P9_33", {
          "AIN4",
          "P9_33",
          "",
          0,
          4,
          4,
          71,
          "",
          0,
          4,
          4096,
          {},
          {}
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P9_35", {
          "AIN6",
          "P9_35",
          "",
          0,
          6,
          6,
          73,
          "",
          0,
          6,
          4096,
          {},
          {}
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P9_36", {
          "AIN5",
          "P9_36",
          "",
          0,
          5,
          5,
          72,
          "",
          0,
          5,
          4096,
          {},
          {}
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P9_37", {
          "AIN2",
          "P9_37",
          "",
          0,
          2,
          2,
          69,
          "",
          0,
          2,
          4096,
          {},
          {}
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P9_38", {
          "AIN3",
          "P9_38",
          "",
          0,
          3,
          3,
          70,
          "",
          0,
          3,
          4096,
          {},
          {}
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P9_39", {
          "AIN0",
          "P9_39",
          "",
          0,
          0,
          0,
          67,
          "",
          0,
          0,
          4096,
          {},
          {}
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P9_40", {
          "AIN1",
          "P9_40",
          "",
          0,
          1,
          1,
          68,
          "",
          0,
          1,
          4096,
          {},
          {}
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P9_41", {
          "CLKOUT2",
          "P9_41",
          "",
          GPIO0,
          20,
          20,
          13,
          "xdma_event_intr1",
          0x1b4,
          {},
          {},
          {},
          (gpioModes) {
            "xdma_event_intr1",
            "N/A",
            "N/A",
            "clkout2",
            "N/A",
            "N/A",
            "N/A",
            "gpio0_20"
          }
        }
      )
    );
    pins.insert (
      std::pair <std::string, const gpioPin> (
        "P9_42", {
          "GPIO0_7",
          "P9_42",
          "",
          GPIO0,
          7,
          7,
          4,
          "ecap0_in_pwm0_out",
          0x164,
          {},
          {},
          (gpioPwm) {
            "ecap0",
            2,
            0,
            0,
            "ecap.0",
            "ECAPPWM0"
          },
          (gpioModes) {
            "eCAP0_in_PWM0_out",
            "uart3_txd",
            "spi1_cs1",
            "pr1_ecap0_ecap_capin_apwm_o",
            "spi1_sclk",
            "mmc0_sdwp",
            "xdma_event_intr2",
            "gpio0_7"
          }
        }
      )
    );

    return pins;
  }

  void memGPIO::mapsMemory () {
    int fd = 0;

    // Need to run this as root, using sudo or su.
    if ((fd = open ("/dev/mem", O_RDWR | O_SYNC)) < 0) {
      perror ("Unable to open '/dev/mem', make sure you have sudo privileges");
      exit (EXIT_FAILURE);
    }

    mapAddress = (volatile unsigned long int *) mmap(
      // Let kernel choose the address at which to create the mapping.
      nullptr,
      // Length bytes starting at offset in the file.
      MMAP_SIZE,
      // The memory protection of the mapping. Read and Write.
      PROT_READ | PROT_WRITE,
      // Flags: Make the map are visible to other processes mapping the same file and
      // guarante to be resident in RAM when the call returns successfully.
      MAP_SHARED | MAP_LOCKED,
      // The file descriptor.
      fd,
      // The length of the mapping.
      MMAP_OFFSET
    );

    // No need to keep fd open after map.
    if(close (fd) < 0){
      perror ("Unable to close /dev/mem file descriptor");
      exit (EXIT_FAILURE);
    }

    // Check if mmap correctly.
    if(mapAddress == MAP_FAILED) {
      perror ("Unable to map /dev/mem");
      exit (EXIT_FAILURE);
    }
  }

  void memGPIO::setLEDPinToDefault (const memGPIO::gpioPin pin) {
    std::string triggerPath;

    // Check if PIN is a USR LED.
    if (pin.led.compare (0, 3, "usr") != 0) {
      std::cout << pin.key << " isn't a USR LED!" << std::endl;
      exit (EXIT_FAILURE);
    }
    // Select the path depending on the kernel version.
    else if (oldKernel) {
      triggerPath += "/sys/class/leds/beaglebone::" + pin.led + "/trigger";
    }
    else {
      triggerPath += "/sys/class/leds/beaglebone:green:" + pin.led + "/trigger";
    }

    if (easyUtils::fileExists (triggerPath)) {
      std::string trigger;

      // Select the trigger depending on the LED number.
      if (pin.key == "USR0") {
        trigger = "heartbeat";
      }
      else if (pin.key == "USR1") {
        trigger = "mmc0";
      }
      else if (pin.key == "USR2") {
        trigger = "cpu0";
      }
      else {
        trigger = "mmc1";
      }

      easyUtils::writeTextFile (triggerPath, trigger);
    }
    else {
      std::cout << "Unable to find the LED trigger path: " << triggerPath << "!" << std::endl;
      exit (EXIT_FAILURE);
    }
  }

  void memGPIO::setLEDPinToGPIO (const memGPIO::gpioPin pin) {
    std::string triggerPath;

    // Check if PIN is a USR LED.
    if (pin.led.compare (0, 3, "usr") != 0) {
      std::cout << pin.key << " isn't a USR LED!" << std::endl;
      exit (EXIT_FAILURE);
    }
    // Select the path depending on the kernel version.
    else if (oldKernel) {
      triggerPath += "/sys/class/leds/beaglebone::" + pin.led + "/trigger";
    }
    else {
      triggerPath += "/sys/class/leds/beaglebone:green:" + pin.led + "/trigger";
    }

    if (easyUtils::fileExists (triggerPath)) {
      easyUtils::writeTextFile (triggerPath, "gpio");
    }
    else {
      std::cout << "Unable to find the LED trigger path: " << triggerPath << "!" << std::endl;
      exit (EXIT_FAILURE);
    }
  }

  void memGPIO::setPinMode (const memGPIO::gpioPin pin, int pinData, std::string pinTemplate) {
    if (oldKernel) {
      std::string muxFile;

      muxFile += "/sys/kernel/debug/omap_mux/" + pin.mux;

      easyUtils::writeTextFile (muxFile, easyUtils::intToHex (pinData));

      if (pinTemplate == "bspwm") {
        std::cout << "PWM is not yet supported!" << std::endl;
        exit (EXIT_FAILURE);

        std::string PWMPath = "/sys/class/pwm/", PWMRequestPath, PWMEnablePath,
                    PWMDutyPath, PWMFreqPath;

        PWMPath += pin.pwm.path;
        PWMRequestPath += PWMPath + "/request";
        PWMEnablePath += PWMPath + "/run";
        PWMDutyPath += PWMPath + "/duty_ns";
        PWMFreqPath += PWMPath + "/period_freq";

        // Clear up any unmanaged usage.
        easyUtils::writeTextFile (PWMRequestPath, "0");
        usleep (1);

        // Reserve use of output.
        easyUtils::writeTextFile (PWMRequestPath, "1");
        // Make sure output is disabled, so it won't start outputing a signal
        // until analogWrite() is called.
        usleep (1);

        easyUtils::writeTextFile (PWMEnablePath, "0");
        // Duty cyle must be set to 0 before changing frequency.
        easyUtils::writeTextFile (PWMDutyPath, "0");
        // Set frequency to default.
        easyUtils::writeTextFile (PWMFreqPath, "0");
      }
    }
    else {
      std::string fragment, dtsFilename, dtboFilename;

      if (pinTemplate == "bspwm") {
        std::cout << "PWM is not yet supported!" << std::endl;
        exit (EXIT_FAILURE);
      }

      fragment += "bspm_" + pin.key + "_" + easyUtils::intToHex (pinData);
      dtsFilename += "/lib/firmware/" + fragment + "-00A0.dts";
      dtboFilename += "/lib/firmware/" + fragment + "-00A0.dtbo";

      // Always create and compile new driver, the values ​​may have changed since
      // the last time.
      createDTS(pin, pinData, dtsFilename);
      createDTBO(dtboFilename, dtsFilename);

      std::string sysDevices = "/sys/devices/";
      std::string boneCapeMgr = easyUtils::findPatternInPath(sysDevices, "bone_capemgr.");

      if (boneCapeMgr.empty ()) {
        std::cout << "Unable to find the bone_capemgr path in " << sysDevices << "!" << std::endl;
        exit (EXIT_FAILURE);
      }
      else {
        std::string slotsFile;
        slotsFile += sysDevices + "/" + boneCapeMgr + "/slots";

        std::string textFile = easyUtils::readTextFile (slotsFile);

        // If the driver is loaded, unloaded it to make sure you have the latest
        // values.
        if (easyUtils::str_match (fragment, textFile)) {
          unloadSlot (slotsFile, fragment);
        }

        // Load the slot.
        easyUtils::writeTextFile (slotsFile, fragment);

        // Check if it has been successfully loaded.
        textFile = easyUtils::readTextFile (slotsFile);
        if (!easyUtils::str_match (fragment, textFile)) {
          std::cout << "Unable to enable the pin: " << pin.key << "!" << std::endl;
          std::cout << "Are you sure that this pin can be used?" << std::endl;
          exit (EXIT_FAILURE);
        }
      }
    }
  }

  void memGPIO::unloadSlot (const std::string slotsFile, const std::string fragment) {
    std::string line, textFile;
    std::istringstream streamText (easyUtils::readTextFile (slotsFile));

    while (!streamText.eof ()) {
      getline (streamText, line);

      if (easyUtils::str_match (fragment, line)) {
        std::size_t pos = line.find(":");
        std::string slot = line.substr (0, pos);

        easyUtils::trim (slot);
        slot = "-" + slot;

        // Unload the slot.
        easyUtils::writeTextFile (slotsFile, slot);

        // Check if it has been successfully unloaded.
        textFile = easyUtils::readTextFile (slotsFile);
        if (easyUtils::str_match (fragment, textFile)) {
          std::cout << "Unable to unload the slot: " << fragment << "!" << std::endl;
          exit (EXIT_FAILURE);
        }
        break;
      }
    }
  }
}
