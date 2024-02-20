Robot CAD: https://cad.onshape.com/documents/422928e74faff55a2b7e3115/w/bd0901fd7d8095d2719307df/e/b6718d1b03bd148cc952edf7

Tasks to complete - 20 February 2024
1. Update Shuffleboard for Tuning Procedure - Open shuffleboard > File > Load Layout > ShuffleboardImport.json within the robot folder
2. Tune Setpoints
    a. In Manual Control, go to each setpoint below and record setpoints. CAD values in degrees are given for reference.
        1. Ground - 0,0 Power On State
        2. Elbow Horizontal (Elbow Down Constraint) - CAD 28
        3. Wrist Constraint - With elbow horizontal, bring wrist in until intake is clear of 12 inch rule. CAD -75
        4. Elbow Vertical (Elbow Up Constraint) - CAD 118
        5. Wrist Stow - Intake a ring, then bring the wrist around until the elbow can be moved in any position without collisions. CAD 180
        6. Elbow Stow/Wrist Shoot - Bring elbow down to shooter, then move wrist out to approximate shooting position. CAD 155/155 
        7. AMP Positions - CAD 110/-10
        8. Trap Scoring - CAD 138/20
        9. Trap Pressure not to be tested yet. Tune at climb test.
    b. Once setpoints have been tuned, progress through preset buttons to make sure the robot does not have clashing problems.
3. Shooter Tuning
    a. Many bounce-outs were seen at week 0 events due to robots shooting too hard with no spin.
        1. Tune shooter speed to be strong but not overpowered.
        2. Tune wheel difference to generate some spin on the ring.
    b. Test feed from intake to make sure intake wheels are not dragging the shot
4. Intake Tuning
    a. Tune Intake Current Limit to intake ring securely while conserving motor life (Motor load + not spinning = hot motor = magic smoke).
5. Light Driver's Practice
