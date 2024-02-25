package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Robot;
import frc.robot.ControlConfigs.PlayerConfigs;

public class IntakeTeleopCommand extends Command{
    
    public IntakeTeleopCommand() {
        addRequirements(Robot.intake);
    }
    
    @Override
    public void execute () {
        if (PlayerConfigs.ground && Robot.intake.intakeState != 3) {
            Robot.intake.intakeState = 1;
        } else if (PlayerConfigs.amp && Robot.intake.intakeState != 3) {
            Robot.intake.intakeState = 2;
        } else if (!Robot.shooter.shootingMode && Robot.intake.intakeState == 0 && (PlayerConfigs.climberDown || PlayerConfigs.climberUp)) {
            //Only climber control can initiate trap position, no manual triggering allowed, must start from stow position
            //To activate trap, disable shooter and run climber down
            Robot.intake.intakeState = 3;
        } else if (PlayerConfigs.fcEnable) {
            //When fine control is enabled, grab current positions to hold
            Robot.intake.intakeState = 4;
            Robot.intake.fcControlElbow = false;
            Robot.intake.fcControlWrist = false;
            Robot.intake.elbowSetPoint = Robot.intake.getElbowEncoder();
            Robot.intake.wristSetPoint = Robot.intake.getWristEncoder();
        } else if (PlayerConfigs.stow) {
            Robot.intake.intakeState = 0;
        }

        //Ground State
        if (Robot.intake.intakeState == 1) {
            //Hold intake to stowed or contrained position to avoid damage or extension rule until low enough to open to ground position
            if (Robot.intake.getElbowEncoder() < IntakeConstants.kElbowUpConstraint) {
                // for the condition ^^^: may need to increase value when bumpers come on
                // was getting caught on just the frame coming down
                SmartDashboard.putString("Wrist Status", "Ground - Success! Setting Ground Position");
                Robot.intake.wristSetPoint = IntakeConstants.kWristGround;
            } else if (Robot.intake.getElbowEncoder() < IntakeConstants.kElbowUpConstraint + 5) {
                SmartDashboard.putString("Wrist Status", "Ground - Elbow Too High, Constrain");
                Robot.intake.wristSetPoint = IntakeConstants.kWristConstraint;
            } else {
                SmartDashboard.putString("Wrist Status", "Ground - Elbow Too High, Stowed");
                Robot.intake.wristSetPoint = IntakeConstants.kWristStowed;
            }
            
            // Only move to ground when wrist is constrained to not break extension rule
            // Prevent fluttering as wrist comes out to ground position by locking elbow setpoint to ground once triggered
            if (Robot.intake.getWristEncoder() < IntakeConstants.kWristConstraint || Robot.intake.elbowSetPoint == IntakeConstants.kElbowGround) {
                SmartDashboard.putString("Elbow Status", "Ground - Success! Setting Ground Position");
                Robot.intake.elbowSetPoint = IntakeConstants.kElbowGround;
            } else {
                SmartDashboard.putString("Elbow Status", "Ground - Wrist Out Of Bounds, Constrain");
                Robot.intake.elbowSetPoint = IntakeConstants.kElbowUpConstraint;
            }  
        //Amp State
        } else if (Robot.intake.intakeState == 2) {
            //Wrist stays in safe position (stowed or constrained) until elbow is in target range (between 115 and 140ish)
            if (Robot.intake.getElbowEncoder() > IntakeConstants.kElbowAmp + 3) {
                SmartDashboard.putString("Wrist Status", "Amp - Elbow Too High, Stowed");
                Robot.intake.wristSetPoint = IntakeConstants.kWristStowed;
            } else if (Robot.intake.getElbowEncoder() < IntakeConstants.kElbowAmp + 15) {
                SmartDashboard.putString("Wrist Status", "Amp - Elbow Too Low, Constrain");
                Robot.intake.wristSetPoint = IntakeConstants.kWristConstraint;
            } else {
                SmartDashboard.putString("Wrist Status", "Amp - Success! Setting Amp Position");
                Robot.intake.wristSetPoint = IntakeConstants.kWristAmp;
            }

            SmartDashboard.putString("Elbow Status", "Amp - Success! Setting Amp Position");
            Robot.intake.elbowSetPoint = IntakeConstants.kElbowAmp;
        //Trap State
        } else if (Robot.intake.intakeState == 3) {
            //Elbow goes to pressure setting if actively climbing. Once high enough to rest on static structural member, elbow moves to trap scoring position
            if (Robot.climbers.getClimberEncoder() > ClimberConstants.scoringHeight) {
                SmartDashboard.putString("Elbow Status", "Trap - Success! Setting Scoring Position");
                Robot.intake.elbowSetPoint = IntakeConstants.kElbowTrapScoring;
            } else {
                SmartDashboard.putString("Elbow Status", "Trap - Climber Too Low, Pressure");
                Robot.intake.elbowSetPoint = IntakeConstants.kElbowTrapPressure;
            }
            
            //Intake stays stowed and out of danger until high enough to rest on static structural member
            if (Robot.climbers.getClimberEncoder() < ClimberConstants.scoringHeight) {
                SmartDashboard.putString("Wrist Status", "Trap - Success! Setting Scoring Position");
                Robot.intake.wristSetPoint = IntakeConstants.kWristTrap;
            } else {
                SmartDashboard.putString("Wrist Status", "Trap - Climber Too Low, Stowed");
                Robot.intake.wristSetPoint = IntakeConstants.kWristStowed;
            }
        //Fine Control
        } else if (Robot.intake.intakeState == 4) {
            //If joysticks are outside of deadband, move individual sections up or down
            //If joysticks are no longer outside of deadband and control is still active, disable control and grab the current position to hold
            //Grabbing position once eliminates gradual sinking due to gravity
            if (Math.abs(PlayerConfigs.fcElbow) > 0.25) {
                SmartDashboard.putString("Elbow Status", "Fine Control, Moving");
                Robot.intake.fcControlElbow = true;
                Robot.intake.elbowSetPoint = Robot.intake.getElbowEncoder() + IntakeConstants.kElbowManualOffset * PlayerConfigs.fcElbow; 
            } else if (Robot.intake.fcControlElbow) {
                SmartDashboard.putString("Elbow Status", "Fine Control, Holding");
                Robot.intake.fcControlElbow = false;
                Robot.intake.elbowSetPoint = Robot.intake.getElbowEncoder();
            }

            if (Math.abs(PlayerConfigs.fcWrist) > 0.25) {
                SmartDashboard.putString("Wrist Status", "Fine Control, Moving");
                Robot.intake.fcControlWrist = true;
                Robot.intake.wristSetPoint = Robot.intake.getWristEncoder() + IntakeConstants.kWristManualOffset * PlayerConfigs.fcWrist;
            } else if (Robot.intake.fcControlWrist) {
                SmartDashboard.putString("Wrist Status", "Fine Control, Holding");
                Robot.intake.fcControlWrist = false;
                Robot.intake.wristSetPoint = Robot.intake.getWristEncoder();
            }
            
        //Stow and Shoot
        } else {
            //Stow wrist if inside extension limit, otherwise hold in constraint position. Only move to shooting position if safe to do so
            if ((Math.abs(IntakeConstants.kElbowStowed - Robot.intake.getElbowEncoder()) < 10) && PlayerConfigs.armScoringMechanism) {
                SmartDashboard.putString("Wrist Status", "Stow - Success! Setting Shooting Position");
                Robot.intake.wristSetPoint = IntakeConstants.kWristShooting;
            } else if (Robot.intake.getElbowEncoder() > IntakeConstants.kElbowDownConstraint) {
                SmartDashboard.putString("Wrist Status", "Stow - Success! Setting Stow Position");
                Robot.intake.wristSetPoint = IntakeConstants.kWristStowed;
            } else {
                SmartDashboard.putString("Wrist Status", "Stow - Elbow Too Low, Constrain Away From Frame");
                Robot.intake.wristSetPoint = IntakeConstants.kWristGround - Robot.intake.getElbowEncoder();
            }
            
            //Only move elbow all the way to stow position if wrist is already in stowed position
            if (Robot.intake.getWristEncoder() > IntakeConstants.kWristStowed - 5 || Robot.intake.elbowSetPoint == IntakeConstants.kElbowStowed) {
                SmartDashboard.putString("Elbow Status", "Stow - Success! Setting Stow Position");
                Robot.intake.elbowSetPoint = IntakeConstants.kElbowStowed;
            } else {
                SmartDashboard.putString("Elbow Status", "Stow - Wrist Exposed, Constrain");
                Robot.intake.elbowSetPoint = IntakeConstants.kElbowUpConstraint;
            }
        }
        
        //Once setpoints have been chosen, pass to controllers
        Robot.intake.setElbowPosition(Robot.intake.elbowSetPoint);
        Robot.intake.setWristPosition(Robot.intake.wristSetPoint);

        //Reject Piece if button is pressed, regardless of intake state
        if (PlayerConfigs.reject) {
            SmartDashboard.putString("Intake Status", "Rejecting");
            Robot.intake.setIntakeVoltage(-12);
        //If intake is on the ground or firing, run at full speed
        } else if (Robot.intake.intakeState == 1 || PlayerConfigs.fire) {
            SmartDashboard.putString("Intake Status", "Intaking");
            Robot.intake.setIntakeVoltage(12);
        //Maintain idle spin for positive hold (think cones sliding out of the intake last year)
        } else {
            SmartDashboard.putString("Intake Status", "Holding");
            Robot.intake.setIntakeVoltage(1);
        }

        SmartDashboard.putNumber("Wrist Setpoint: ", Robot.intake.wristSetPoint);
        SmartDashboard.putNumber("Elbow Setpoint: ", Robot.intake.elbowSetPoint);
        SmartDashboard.putNumber("Wrist Position: ", Robot.intake.getWristEncoder());
        SmartDashboard.putNumber("Elbow Position: ", Robot.intake.getElbowEncoder());
        SmartDashboard.putNumber("Intake State: ",Robot.intake.intakeState);
        SmartDashboard.putBoolean("Piece Acquired: ", Robot.intake.getIntakeCurrent() > 20);
        SmartDashboard.putNumber("Intake Current: ", Robot.intake.getIntakeCurrent()); //<-Valueable for testing, can be removed before competition if space is needed.

        Robot.intake.intakeLog();
    }
}
