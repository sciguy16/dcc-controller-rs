# dcc-controller-rs
Open source lightweight DCC controller written in Rust

## Features
* Two control potentiometers to control two locomotives, with short-mode
addresses selected via dipswitches
* Potentiometers are centre-off; rotate left to drive the corresponding
locomotive in reverse, and right to drive it forwards
* Forward (green) and reverse (red) LED for each channel to show when it
is active
* On-off-on mini slide switch to select programming mode
  * programming mode disables all control functions and just sends out
DCC IDLE packets until the "program" button is pressed
  * Controller will then send out the programming sequence to program the
locomotive with the address set on the selected channel
* Current measurement and overcurrent shut-off
* Overcurrent alam LED
* Optional additional buttons which may be configured to control e.g.
horn/whistle or other accessory functions
* Optional serial/USB/RS485 hardware for integration with other control
systems
* Screw terminals to connect power supply and track outputs
* SMD power MOSFETs for compactness
* Pure-Rust open source firmware based on dcc-rs
* Open source hardware

# License
Source code for the software components is distributed under the terms
of the Mozilla Public License Version 2.0.

Design files for the hardware components are distributed under the terms
of the CERN Open Hardware Licence Version 2 - Weakly Reciprocal.

