FRC 7506 Robot Summary

Mechanisms

INTAKE	

Motors

1. Elbow Rotator Left

2. Elbow Rotator Right
 
3. Wrist Rotator

4. Intake Roller Left

5. Intake Roller Right
	
Methods

1. setElbowPosition(double position)

2. setWristPosition(double position)

3. setIntakeAction(double velocity)
	
TeleOp Commands

1. Movement Commands

	A. Ground Pickup

	- Elbow and Wrist in position

  	B. Human Player Pickup

	- Same as above
	
 	C. Amp Scoring
	
 	- Everything to position. If releasing, drop the intake a little to make sure it goes in, see setpoints below
	
 	D. Climb sequence
	
 	- Link Trap Setpoints to climber movement
		
  2. Intake Commands
			
   A. Ground/Human Player Pickup
   
   - Intake running, need a flag to stop the intake if there is a current spike (got a piece)
   
   -   One button should do this and 1.A/1.B
			
   B. Releasing
   
   - Press button, ring goes out the back
	
Shooter

	
 Motors
		
  1. Flywheel Left
		
  2. Flywheel Right
	
 Methods
		
  1. setTriggerVoltage(double volts)

  2. setFlywheelVoltage(double volts)

 TeleOp Commands

  1. Trigger On/Off

  2. Flywheel Speed up or idle speed (Flywheel should never fully stop spinning until climbers are activated)



Climber

 Motors

  1. Left Climber

  2. Right Climber

 Methods

  setClimberPositions(double position)

 TeleOp Commands

  1. Climb Up/Down

   - We will need to pull all the way to the frame. Find the setpoints and hit them every time



Setpoints - VERIFY THESE ONCE THE ROBOT IS BUILT

GROUND

 1. Elbow 0

 2. Wrist 0

TRAVELING (Not to Speaker)

 1. Wrist -40

TRAVELING (To Speaker)

 1. Wrist 105

 2. Elbow doesn't move until Wrist is done

AMP

 1. Elbow 100

 2. Wrist -10
    
SOURCE

 1. Elbow 160

 2. Wrist -70

SPEAKER/STOWED

 1. Elbow 155

 2. Wrist 135

CLIMB UP

 1. Climbers 0

CLIMB DOWN

 2. Climbers 26
