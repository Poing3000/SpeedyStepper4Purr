# SpeedyStepper4Purr - Arduino Library

This library is forked from the great SpeedyStepper library created by S. Reifel & Co. The changes introduced adapt the library for the use of stepper motors in the PurrPleaser3000 projects (https://github.com/Poing3000/PurrPleaser3000). Hence, it is specially tailored – however, it may still be suitable for other projects.

## Changes implemented:
→ Reduced to "steps/second" functions (deleted per Revolution and Distance functions)

→ Adapted Homing function to (almost) non - blocking code, advanced error detection/handling
  and the possibility to change end stop inputs (i.e., from an end stop to driver stall detection).

NOTE, if the end stop pin (homeEndStopNumber) is set to 99, the end stop signal is expected from an external source / function (see move home).
NOTE, error handling is BLOCKING code.
WARNING, limited to 4 steppers, as only 4 interrupt pins are available (see switch below).

## Documentation:
Please find the superior documentation at the original repository from [S. Reifel](https://github.com/Stan-Reifel/SpeedyStepper). However, please note that certain functions are no longer available due to the changes mentioned above.
