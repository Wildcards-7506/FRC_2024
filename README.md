Tasks to complete - 3 February 2024
1. Link Right Elbow to Left
    a. Check with Rev HW Client, the setting may not have stuck on startup.
    b. Check that the Right Elbow is following the correct CAN ID (9)
    b. Right elbow MUST be reversed, motors should rotate towards and away from one another, never in the same direction
2. PID Tuning
    a. Robot.java has been set up to adjust P values on the fly through the smart dashboard. Tune Elbow, Wrist, and Shooter using these
    b. Once good P values are found, replace Constants and change variables to final.
3. On-Bot Intake Test
    a. Left Elbow goes on the left hand side when standing where the intake should land. 
    b. Wrist appears to be set up to go on the right hand side.
    c. Remember, the motor axles are not moving, the motors themselves are. Motion is reversed from what the test board is showing you
4. Auto Path Verification
    a. Make sure the robot can handle new auto paths running faster.
    b. Trident, Jaguar Up, and Jaguar Down are probably the only routines you can run on that small section of carpet, All others go to the centerline. Maybe we get lucky, but check first.