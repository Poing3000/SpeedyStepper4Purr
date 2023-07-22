
//      ******************************************************************
//      *                                                                *
//      *                      Speedy Stepper 4 Purr                     *
//      *								 *
//      *            Poing3000                       20/07/2023          *
//      *                   Copyright (c) Poing3000, 2023                *
//      *								 *
//      *			   Original from			 *
//      *                   Speedy Stepper Motor Driver                  *
//      *                                                                *
//      *            Stan Reifel                     12/8/2014           *
//      *               Copyright (c) S. Reifel & Co, 2014               *
//      *                                                                *
//      ******************************************************************


// MIT License
// 
// Updates Copyright (c) 2023 Poing3000
// Original Copyright (c) 2014 Stanley Reifel & Co.
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is furnished
// to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


//
// This library is used to control stepper motors in the PurrPleaser3000 Cat Feeding Machine Projects.
// For further info on the usage of this libray please find the superior documentation at the original
// repository "SpeedyStepper4Purr" library from S. Reifel.
//
// Changes implemented:
// > Reduced to "steps/second" functions (deleted per Revolution and Distance functions).
// > Adapted Homing function to (almost) non - blocking code, advanced error detection/handling
//	 and the possibility to change end stop inputs (i.e., from an end stop to driver stall detection).
//	> NOTE, the end stop pin must now be declared as an INPUT pin in the main code.
//  > NOTE, error handling is BLOCKING code.

// =====================================================================================================


#include "SpeedyStepper4Purr.h"


// ---------------------------------------------------------------------------------
//                                  Setup functions 
// ---------------------------------------------------------------------------------


// Constructor for the stepper class
SpeedyStepper4Purr::SpeedyStepper4Purr(const byte whichDiag) : whichDiag_ (whichDiag)
{
  // initialize constants
  stepPin = 0;
  directionPin = 0;
  homeEndStop = 0;
  homeDiagPin = 0;
  currentPosition_InSteps = 0;
  desiredSpeed_InStepsPerSecond = 200.0;
  acceleration_InStepsPerSecondPerSecond = 200.0;
  currentStepPeriod_InUS = 0.0;
  flag_prepareHoming = false;

  flagStalled_ = false;

}

//  Connect the stepper object to the IO pins
//  Enter:  stepPinNumber = IO pin number for the Step
//          directionPinNumber = IO pin number for the direction bit
// 			homeEndStopNumber = IO pin number for the home limit switch
// 			homediagPinNumber = IO pin number for the driver stall detection.
//				>>NOTE: this limits the number of steppers to 3, as only 3
//						interrupt pins are available (see switch below).
//
void SpeedyStepper4Purr::connectToPins(byte stepPinNumber, byte directionPinNumber, byte homeEndStopNumber, byte homeDiagPinNumber)
{
  // Remember the pin numbers
  stepPin = stepPinNumber;
  directionPin = directionPinNumber;
  homeEndStop = homeEndStopNumber;
  homeDiagPin = homeDiagPinNumber;
  
  // Configure the IO bits
  pinMode(stepPin, OUTPUT);
  digitalWrite(stepPin, LOW);

  pinMode(directionPin, OUTPUT);
  digitalWrite(directionPin, LOW);

  pinMode(homeEndStop, INPUT_PULLUP);

  //Assign interrupts for stall detection
  switch (whichDiag_) {
	case 0:
		attachInterrupt(digitalPinToInterrupt(homeDiagPin), StallInterrupt0, RISING);
		instance0_ = this;
		break;
	case 1:
		attachInterrupt(digitalPinToInterrupt(homeDiagPin), StallInterrupt1, RISING);
		instance1_ = this;
		break;
	case 2:
		attachInterrupt(digitalPinToInterrupt(homeDiagPin), StallInterrupt2, RISING);
		instance2_ = this;
		break;
  }
}

//Interrupt glue routines
void SpeedyStepper4Purr::StallInterrupt0() {
	instance0_->StallIndication();
}

void SpeedyStepper4Purr::StallInterrupt1() {
	instance1_->StallIndication();
}

void SpeedyStepper4Purr::StallInterrupt2() {
	instance2_->StallIndication();
}

//for use by interrupt glue routines
SpeedyStepper4Purr * SpeedyStepper4Purr::instance0_;
SpeedyStepper4Purr * SpeedyStepper4Purr::instance1_;
SpeedyStepper4Purr * SpeedyStepper4Purr::instance2_;

void SpeedyStepper4Purr::StallIndication() {
	flagStalled_ = true;
}


// ---------------------------------------------------------------------------------
//									Public functions
// ---------------------------------------------------------------------------------


//
// Set the current position of the motor, this does not move the motor
// Note: This function should only be called when the motor is stopped
//	Enter:  currentPositionInSteps = the new position of the motor in steps
//
void SpeedyStepper4Purr::setCurrentPositionInSteps(long currentPositionInSteps)
{
  currentPosition_InSteps = currentPositionInSteps;
}


//
// Get the current position of the motor, this functions is updated
// while the motor moves
//	Exit:  a signed motor position in steps returned
//
long SpeedyStepper4Purr::getCurrentPositionInSteps()
{
  return(currentPosition_InSteps);
}

/*NEEDED/DELETE
//
// Setup a "Stop" to begin the process of decelerating from the current velocity to  
// zero, decelerating requires calls to processMove() until the move is complete.
//
void SpeedyStepper4Purr::setupStop()
{
  //
  // Move the target position so that the motor will begin deceleration now.
  //
  if (direction_Scaler > 0)
    targetPosition_InSteps = currentPosition_InSteps + decelerationDistance_InSteps;
  else
    targetPosition_InSteps = currentPosition_InSteps - decelerationDistance_InSteps;
}
*/

//
// Set the maximum speed, this is the maximum speed reached  
// while accelerating
// Note: this can only be called when the motor is stopped
//	Enter:  speedInStepsPerSecond = speed to accelerate up to, units in steps/second
//
void SpeedyStepper4Purr::setSpeedInStepsPerSecond(float speedInStepsPerSecond)
{
  desiredSpeed_InStepsPerSecond = speedInStepsPerSecond;
}


//
// Set the rate of acceleration.
// Note: this can only be called when the motor is stopped
//  Enter:  accelerationInStepsPerSecondPerSecond = rate of acceleration, units in 
//          steps/second/second
//
void SpeedyStepper4Purr::setAccelerationInStepsPerSecondPerSecond(
                      float accelerationInStepsPerSecondPerSecond)
{
    acceleration_InStepsPerSecondPerSecond = accelerationInStepsPerSecondPerSecond;
}


// HOMING:
// Home the motor by moving until the homing sensor is activated, then set the 
// position to zero.
//  Enter:  directionTowardHome = 1 to move in a positive direction, -1 to move in 
//             a negative directions.
//          speedInStepsPerSecond = speed to accelerate up to while moving toward 
//             home.
//          maxDistanceToMoveInSteps = unsigned maximum distance to move toward 
//             home before giving up.
//          homeEndstop = true if the homing sensor is an endstop switch,
//			   false for fallbacl method (i.e. by stall detection). 
//  Exit:   0 - returned if still homing
//			1 - returned if successful
// 			2 - returned if aborted due to error: "Enstop always triggered".
//			3 - returned if aborted due to error: "Enstop not triggered".
// 			4 - big error, this shoud not happen.

bool SpeedyStepper4Purr::getEndstops(bool whichEndstop) {
	bool setEndStop;
	if (whichEndstop) {
		if (digitalRead(homeEndStop) == HIGH) {
			setEndStop = true;
		}
		else {
			setEndStop = false;
		}
	}
	else {
		setEndStop = flagStalled_;
		flagStalled_ = false;
	}

	return setEndStop;
}


byte SpeedyStepper4Purr::moveToHome(long directionTowardHome,
	float speedInStepsPerSecond, long maxDistanceToMoveInSteps, bool useHomeEndStop)
{
	bool EndStop;
	EndStop = getEndstops(useHomeEndStop);
	
	// Prepare for homing, else do homing.
	if (!flag_prepareHoming){
		setSpeedInStepsPerSecond(speedInStepsPerSecond);

		//BLOCKING
		//-----------------------------------------------------------------------------------
		//Move once away from Endstop in case that it is allready in the target zone.
		if (EndStop == true) {
			// 10% of maxDistanceToMoveInSteps should be enough to get away from the endstop.
			setupRelativeMoveInSteps(maxDistanceToMoveInSteps * directionTowardHome * -0.1);
			while (!processMovement()) {
				if (getEndstops(useHomeEndStop) == false) {
					break;
				}
			}
		//-----------------------------------------------------------------------------------
		
		// Check if endstop has actually reacted
			if (getEndstops(useHomeEndStop) == true)
				return(2);
		}
		setupRelativeMoveInSteps(maxDistanceToMoveInSteps * directionTowardHome);
		flag_prepareHoming = true;
		return(0);
	}

	// Do homing
	else {	
		if (EndStop == false) {
			if (processMovement() == true) {
				//Error, endstop not found
				return(3);
			}
			else {
				//Still homing
				return(0);
			}
		}
		else {
			if (!useHomeEndStop) {
				//BLOCKING
				//-----------------------------------------------------------------------------------
				//Move to the final start position, since we are homing with stall.
				setupRelativeMoveInSteps(200 * directionTowardHome * -1);
				while (!processMovement());
			}
			//-----------------------------------------------------------------------------------

			// Successfully homed, set the current position to 0
			setCurrentPositionInSteps(0L);
			flag_prepareHoming = false;
			return(1);
		}
	}
	// This should never be reached.
	return(4);
}

// STEPPER ERROR HANDLING:
// This function is to recover from a stepper error.
//  Enter:  error code to start relevant error handling.
//
//  Exit:	0 - error solved
//			1 - unknown drivetrain malfunction
//			2 - slider is stuck
//			3 - endstop malfunction

byte SpeedyStepper4Purr::ErrorHandling(byte error, long directionTowardHome,
	float speedInStepsPerSecond, long maxDistanceToMoveInSteps) {
	switch (error) {
		// BLOCKING
		// 1 - SOLVE HOMING ERROR
		case 1:
		// ===========================================================================================
		// If stall is true, either there is an endstop malfunction or the slider is stuck.
		// (But if stall is not true, than there is an unknown drivetrain error .)
		// So stall flag is reset to then check if stall apears again while trying to move the slider.
		// If stall is again true, the slider is stuck and we need to try to free it up.
		// If stall is false/we got to get the slider unstuck, we can try to home again.
		// If this fails again, the endstop may not working, so we can try to home with stall.
		// -------------------------------------------------------------------------------------------

		if (flagStalled_) {
			flagStalled_ = false;
			long travelDistance = maxDistanceToMoveInSteps * directionTowardHome * -0.1;
			moveRelativeInSteps(travelDistance);
			if (flagStalled_) {
				//Slider stuck, try to free it up.
				int i = 1, y = 1;
				setSpeedInStepsPerSecond(speedInStepsPerSecond * 0.1);	// Reduce to generate more torque.

				// Vibrate slider to free up.
				// ---------------------------------------------------
				while (i <= travelDistance) {
					moveRelativeInSteps(i);
					if (i > 0) {
						i = -i;
					}
					else {

						if (y < 200) {
							i = -i + 1;
							y++;
						}
						else {
							i = -i + y;
						}
						setSpeedInStepsPerSecond(speedInStepsPerSecond * 0.1 * y);
					}
				}
				// ---------------------------------------------------
				setSpeedInStepsPerSecond(speedInStepsPerSecond);	// Reset speed.

				// Check if slider is free now.
				flagStalled_ = false;
				moveRelativeInSteps(travelDistance);
				if (flagStalled_ == true) {
					// Slider still stuck >> EMGY mode.
					return(2);
				}
				// Slider is free (we can try to home again).
				return (0);
			}
			else {
				// Possible endstop malfunction (try to home with stall).
				return(3);
			}
		}
		else {
			// If stall is false, neither the endstop works nor the stall detection.
			// Unknown drivetrain malfunction >>EMGY mode.
			return(1);
		}
	}
	return (0);
}

//
// move relative to the current position, units are in steps, this function does 
// not return until the move is complete
//  Enter:  distanceToMoveInSteps = signed distance to move relative to the current 
//            position in steps
//
void SpeedyStepper4Purr::moveRelativeInSteps(long distanceToMoveInSteps)
{
  setupRelativeMoveInSteps(distanceToMoveInSteps);
  
  while(processMovement() != 1);
}


//
// setup a move relative to the current position, units are in steps, no motion  
// occurs until processMove() is called.  Note: this can only be called when the 
// motor is stopped
//  Enter:  distanceToMoveInSteps = signed distance to move relative to the current  
//          position in steps
//
void SpeedyStepper4Purr::setupRelativeMoveInSteps(long distanceToMoveInSteps)
{
  setupMoveInSteps(currentPosition_InSteps + distanceToMoveInSteps);
}

/*DELETE BLOCKING?
//
// move to the given absolute position, units are in steps, this function does not 
// return until the move is complete
//  Enter:  absolutePositionToMoveToInSteps = signed absolute position to move to  
//            in units of steps
//
void SpeedyStepper4Purr::moveToPositionInSteps(long absolutePositionToMoveToInSteps)
{
  setupMoveInSteps(absolutePositionToMoveToInSteps);
  
  while(processMovement() != 1)
    ;
}
*/

//
// setup a move, units are in steps, no motion occurs until processMove() is called
// Note: this can only be called when the motor is stopped
//  Enter:  absolutePositionToMoveToInSteps = signed absolute position to move to in 
//          units of steps
//
void SpeedyStepper4Purr::setupMoveInSteps(long absolutePositionToMoveToInSteps)
{
  long distanceToTravel_InSteps;
  
  
  //
  // save the target location
  //
  targetPosition_InSteps = absolutePositionToMoveToInSteps;
  

  //
  // determine the period in US of the first step
  //
  ramp_InitialStepPeriod_InUS =  1000000.0 / sqrt(2.0 * 
                                    acceleration_InStepsPerSecondPerSecond);
    
    
  //
  // determine the period in US between steps when going at the desired velocity
  //
  desiredStepPeriod_InUS = 1000000.0 / desiredSpeed_InStepsPerSecond;


  //
  // determine the number of steps needed to go from the desired velocity down to a 
  // velocity of 0, Steps = Velocity^2 / (2 * Accelleration)
  //
  decelerationDistance_InSteps = (long) round((desiredSpeed_InStepsPerSecond * 
    desiredSpeed_InStepsPerSecond) / (2.0 * acceleration_InStepsPerSecondPerSecond));
  
  
  //
  // determine the distance and direction to travel
  //
  distanceToTravel_InSteps = targetPosition_InSteps - currentPosition_InSteps;
  if (distanceToTravel_InSteps < 0) 
  {
    distanceToTravel_InSteps = -distanceToTravel_InSteps;
    direction_Scaler = -1;
    digitalWrite(directionPin, HIGH);
  }
  else
  {
    direction_Scaler = 1;
    digitalWrite(directionPin, LOW);
  }


  //
  // check if travel distance is too short to accelerate up to the desired velocity
  //
  if (distanceToTravel_InSteps <= (decelerationDistance_InSteps * 2L))
    decelerationDistance_InSteps = (distanceToTravel_InSteps / 2L);


  //
  // start the acceleration ramp at the beginning
  //
  ramp_NextStepPeriod_InUS = ramp_InitialStepPeriod_InUS;
  acceleration_InStepsPerUSPerUS = acceleration_InStepsPerSecondPerSecond / 1E12;
  startNewMove = true;
}



//
// if it is time, move one step
//  Exit:  true returned if movement complete, false returned not a final target 
//           position yet
//
bool SpeedyStepper4Purr::processMovement(void)
{ 
  unsigned long currentTime_InUS;
  unsigned long periodSinceLastStep_InUS;
  long distanceToTarget_InSteps;

  //
  // check if already at the target position
  //
  if (currentPosition_InSteps == targetPosition_InSteps)
    return(true);

  //
  // check if this is the first call to start this new move
  //
  if (startNewMove)
  {    
    ramp_LastStepTime_InUS = micros();
    startNewMove = false;
  }
    
  //
  // determine how much time has elapsed since the last step (Note 1: this method   
  // works even if the time has wrapped. Note 2: all variables must be unsigned)
  //
  currentTime_InUS = micros();
  periodSinceLastStep_InUS = currentTime_InUS - ramp_LastStepTime_InUS;

  //
  // if it is not time for the next step, return
  //
  if (periodSinceLastStep_InUS < (unsigned long) ramp_NextStepPeriod_InUS)
    return(false);

  //
  // determine the distance from the current position to the target
  //
  distanceToTarget_InSteps = targetPosition_InSteps - currentPosition_InSteps;
  if (distanceToTarget_InSteps < 0) 
    distanceToTarget_InSteps = -distanceToTarget_InSteps;

  //
  // test if it is time to start decelerating, if so change from accelerating to 
  // decelerating
  //
  if (distanceToTarget_InSteps == decelerationDistance_InSteps)
    acceleration_InStepsPerUSPerUS = -acceleration_InStepsPerUSPerUS;
  
  //
  // execute the step on the rising edge
  //
  digitalWrite(stepPin, HIGH);
  
  //
  // delay set to almost nothing because there is so much code between rising and 
  // falling edges
  delayMicroseconds(2);        
  
  //
  // update the current position and speed
  //
  currentPosition_InSteps += direction_Scaler;
  currentStepPeriod_InUS = ramp_NextStepPeriod_InUS;


  //
  // compute the period for the next step
  // StepPeriodInUS = LastStepPeriodInUS * 
  //   (1 - AccelerationInStepsPerUSPerUS * LastStepPeriodInUS^2)
  //
  ramp_NextStepPeriod_InUS = ramp_NextStepPeriod_InUS * 
    (1.0 - acceleration_InStepsPerUSPerUS * ramp_NextStepPeriod_InUS * 
    ramp_NextStepPeriod_InUS);


  //
  // return the step line high
  //
  digitalWrite(stepPin, LOW);
 
 
  //
  // clip the speed so that it does not accelerate beyond the desired velocity
  //
  if (ramp_NextStepPeriod_InUS < desiredStepPeriod_InUS)
    ramp_NextStepPeriod_InUS = desiredStepPeriod_InUS;


  //
  // update the acceleration ramp
  //
  ramp_LastStepTime_InUS = currentTime_InUS;
 
 
  //
  // check if move has reached its final target position, return true if all done
  //
  if (currentPosition_InSteps == targetPosition_InSteps)
  {
    currentStepPeriod_InUS = 0.0;
    return(true);
  }
    
  return(false);
}



//
// Get the current velocity of the motor in steps/second.  This functions is updated
// while it accelerates up and down in speed.  This is not the desired speed, but  
// the speed the motor should be moving at the time the function is called.  This  
// is a signed value and is negative when the motor is moving backwards.
// Note: This speed will be incorrect if the desired velocity is set faster than
// this library can generate steps, or if the load on the motor is too great for
// the amount of torque that it can generate.
//  Exit:  velocity speed in steps per second returned, signed
//
float SpeedyStepper4Purr::getCurrentVelocityInStepsPerSecond()
{
  if (currentStepPeriod_InUS == 0.0)
    return(0);
  else
  {
    if (direction_Scaler > 0)
      return(1000000.0 / currentStepPeriod_InUS);
    else
      return(-1000000.0 / currentStepPeriod_InUS);
  }
}



//
// check if the motor has completed its move to the target position
// Exit: true returned if the stepper is at the target position
//
bool SpeedyStepper4Purr::motionComplete()
{
  if (currentPosition_InSteps == targetPosition_InSteps)
    return(true);
  else
    return(false);
}

// -------------------------------------- End --------------------------------------

