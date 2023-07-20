# SpeedyStepper4Purr - Arduino Library

This library is forked from the great SpeedyStepper library created by S. Reifel & Co. The changes introduced adapt the library for the use of stepper motors in the PurrPleaser3000 projects. Hence, it is specially tailored – however, it may still be suitable for other projects.

## Changes implemented:
→ Reduced to "steps/second" functions (deleted per Revolution and Distance functions)
→ Adapted Homing function to non-blocking code, advanced error detection/handling and the possibility to change end stop inputs (i.e., from an end stop to driver stall detection).

## Documentation:
Please find the superior documentation at the original repository from [S. Reifel](https://github.com/Stan-Reifel/SpeedyStepper). However, please note that certain functions are no longer available due to the changes mentioned above.

## To do:
- [ ] Adapt library
	- [ ] Delete not needed parts (rev&distance functionality)
	- [ ] Update Homing 
		- [ ] Make it non-blocking
		- [ ] Implement advanced error handling
		- [ ] Implement flexible endstops (Stall endstop)
- [ ] Include brief documentation for the changes introduced.


