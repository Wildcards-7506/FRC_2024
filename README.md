Robot CAD: https://cad.onshape.com/documents/422928e74faff55a2b7e3115/w/bd0901fd7d8095d2719307df/e/b6718d1b03bd148cc952edf7

Tasks to complete - 27 February 2024
1. Update Shuffleboard for Tuning Procedure - Open shuffleboard > File > Load Layout > ShuffleboardImport.json within the robot folder.
2. Change Intake Motor Breaker on the PDP to a 40A Breaker if not done already.
3. Trap test (Jayden - OPERATOR, Ricardo - DRIVER, Ryan - PROGRAMMER, Herron - MENTOR or suitable alternative)
    A. PROGRAMMER - Enable in STOW position.
    B. OPERATOR - Disable shooter if it is spinning (Right Bumper).
    C. OPERATOR - Climbers down SLIGHTLY (D-Pad Down - Elbow will rotate to TRAP PRESSURE position, WRIST SHOULD NOT MOVE).
    D. MENTOR - Once elbow is in satisfactory position, push up on the elbow to try and tip the robot backwards.
    E. PROGRAMMER - When robot is tipped onto two wheels, record difference between set point and position. This is the offset we will see due to gravity when climbing and needs to be added to TRAP PRESSURE constant.
    F. MENTOR - Let robot back down onto four wheels and clear robot.
    G. OPERATOR - Climber DOWN all the way (D-Pad Down).
    H. PROGRAMMER - Keep an eye on climber value, when climber is past scoring value (Currently 23), the elbow should move back towards the shooter slightly and the intake should move to scoring position.
    I. DRIVER - When in scoring position, press FIRE button to test intake.
    J. OPERATOR - return intake to STOW position.
4. Shooter Tuning
    a. Many bounce-outs were seen at week 0 events due to robots shooting too hard with no spin.
        1. Tune shooter speed to be strong but not overpowered.
        2. Tune wheel difference to generate some spin on the ring.
5. Light Driver's Practice
