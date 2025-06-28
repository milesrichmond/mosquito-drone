# Mosquito-Drone

## Project overview

A custom-built drone project, where every piece (within reason) is built from the
ground up. This project also serves as a first foray into embedded systems for the
two of us.

### Materials

We're currently basing the project off an STM32 microcontroller (specifically the
stm32f103c8t6) paired with the BNO055 IMU from adafruit. The drone chasis will be
somewhere around the 3S category, but we have not yet taken the steps to finalize
that decision.

## Progress

The following features are implemented, but are still subject to change:

- Basic I2C communications between a single mcu and its peripherals.
- Semihosting support for debugging.
- Orientation readings from the BNO055.
- Pressue altitude estimations from the BMP390.
- Temperature readings from the BMP390 (Also available on the BNO055, but untested).

## The next steps

- Flight control behaviors and parameters.
  - Dampening of corrections to remove oscillations and overcorrections.
  - Testing to ensure these behaviors are what we're looking for
- ESC interface.
  - PWM output from the mcu.
- *Battery power & voltage regulation*
