# SpeedyStepper4Purr - Arduino Library

This library is forked from the great SpeedyStepper library created by S. Reifel & Co. The changes introduced adapt the library for the use of stepper motors in the PurrPleaser3000 projects. Hence, it is specially tailored – however, it may still be suitable for other projects.

## Changes implemented:
→ Reduced to "steps/second" functions (deleted per Revolution and Distance functions)

→ Adapted Homing function to non-blocking code, advanced error detection/handling and the possibility to change end stop inputs (i.e., from an end stop to driver stall detection).

## Documentation:
Please find the superior documentation at the original repository from [S. Reifel](https://github.com/Stan-Reifel/SpeedyStepper). However, please note that certain functions are no longer available due to the changes mentioned above.

## To do:
- [ ] Adapt library
	- [x] Delete not needed parts (rev&distance functionality)
	- [x] Update Homing 
		- [x] Make it non-blocking
		- [x] Implement homing error handling
		- [x] Implement flexible endstops (Stall endstop)
	- [x] Include further error handling, if needed (e.g. to handle feeding errors).
  	- [ ] Cleanup
- [ ] Include brief documentation for the changes introduced.


