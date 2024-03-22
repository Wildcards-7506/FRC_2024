package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Robot;
import frc.robot.ControlConfigs.PlayerConfigs;

public class IntakeTeleopCommand extends Command{
    
    public IntakeTeleopCommand() {
        addRequirements(Robot.intake);
    }
    
    @Override
    public void execute () {
        if (PlayerConfigs.ground) {
            Robot.intake.intakeState = 1;
        } else if (PlayerConfigs.amp) {
            Robot.intake.intakeState = 2;
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
        if (!Robot.shooter.shootingMode || Robot.intake.intakeState == 1) {
            //Hold intake to stowed or contrained position to avoid damage or extension rule until low enough to open to ground position
            if (Robot.intake.getElbowEncoder() < IntakeConstants.kElbowDownConstraint + 5) {
                Robot.intake.wristSetPoint = IntakeConstants.kWristGround;
            } else if (Robot.intake.getElbowEncoder() < IntakeConstants.kElbowUpConstraint - 2) {
                Robot.intake.wristSetPoint = IntakeConstants.kWristConstraint - 5;
            } else {
                Robot.intake.wristSetPoint = Robot.intake.getWristEncoder();
            }

            if (Robot.intake.getWristEncoder() < IntakeConstants.kWristConstraint + 10 || Robot.intake.wristSetPoint == IntakeConstants.kWristGround) {
                Robot.intake.elbowSetPoint = IntakeConstants.kElbowGround;
            } else {
                Robot.intake.elbowSetPoint = IntakeConstants.kElbowUpConstraint - 5;
            }
        //Amp State
        } else if (Robot.intake.intakeState == 2) {
            if (Robot.intake.getElbowEncoder() > IntakeConstants.kElbowAmp + 5) {
                Robot.intake.wristSetPoint = IntakeConstants.kWristStowed;
            } else {
                Robot.intake.wristSetPoint = IntakeConstants.kWristAmp;
            }
            Robot.intake.elbowSetPoint = IntakeConstants.kElbowAmp;
        //Fine Control
        } else if (Robot.intake.intakeState == 4) {
            if (Math.abs(PlayerConfigs.fcElbow) > 0.25) {
                Robot.intake.fcControlElbow = true;
                Robot.intake.elbowSetPoint = Robot.intake.getElbowEncoder() + IntakeConstants.kElbowManualOffset * PlayerConfigs.fcElbow;
            } else if (Robot.intake.fcControlElbow) {
                Robot.intake.fcControlElbow = false;
                Robot.intake.elbowSetPoint = Robot.intake.getElbowEncoder();
            }

            if (Math.abs(PlayerConfigs.fcWrist) > 0.25) {
                Robot.intake.fcControlWrist = true;
                Robot.intake.wristSetPoint = Robot.intake.getWristEncoder() + IntakeConstants.kWristManualOffset * PlayerConfigs.fcWrist;
            } else if (Robot.intake.fcControlWrist) {
                Robot.intake.fcControlWrist = false;
                Robot.intake.wristSetPoint = Robot.intake.getWristEncoder();
            }
            
        //Stow, Shoot, and Climb
        } else {            
            if (Robot.intake.getElbowEncoder() < IntakeConstants.kElbowUpConstraint - 10) {
                Robot.intake.wristSetPoint = IntakeConstants.kWristConstraint;
            } else if (PlayerConfigs.armScoringMechanism) {
                Robot.intake.wristSetPoint = IntakeConstants.kWristShooting;
            } else {
                Robot.intake.wristSetPoint = IntakeConstants.kWristStowed;
            }

            if (Robot.intake.getWristEncoder() > IntakeConstants.kWristShooting - 10 || Robot.intake.getWristEncoder() > IntakeConstants.kWristStowed - 10) {
                Robot.intake.elbowSetPoint = IntakeConstants.kElbowStowed;
            } else {
                Robot.intake.elbowSetPoint = IntakeConstants.kElbowUpConstraint;
            }
        }
        
        //Once setpoints have been chosen, pass to controllers
        Robot.intake.setElbowPosition(Robot.intake.elbowSetPoint);
        Robot.intake.setWristPosition(Robot.intake.wristSetPoint);

        //Reject Piece if button is pressed, regardless of intake state
        if (PlayerConfigs.intake && Robot.intake.intakeState == 1) {
            Robot.intake.setIntakeCurrentLimit(IntakeConstants.kIntakeCurrentLimit);
            Robot.intake.setIntakeVoltage(12,-2.4);
        } else if (PlayerConfigs.intake) {
            Robot.intake.setIntakeCurrentLimit(IntakeConstants.kIntakeCurrentLimit);
            Robot.intake.setIntakeVoltage(2,12);
        } else if (PlayerConfigs.fire) {
            Robot.intake.setIntakeCurrentLimit(IntakeConstants.kIntakeShootingLimit);
            Robot.intake.setIntakeVoltage(12,12);
        } else if (PlayerConfigs.reject) {
            Robot.intake.setIntakeCurrentLimit(IntakeConstants.kIntakeShootingLimit);
            Robot.intake.setIntakeVoltage(-2.4,-12);
        } else {
            Robot.intake.setIntakeCurrentLimit(IntakeConstants.kIntakeCurrentLimit);
            Robot.intake.setIntakeVoltage(0,0);
        }

        SmartDashboard.putNumber("Wrist Setpoint: ", Robot.intake.wristSetPoint);
        SmartDashboard.putNumber("Elbow Setpoint: ", Robot.intake.elbowSetPoint);
        SmartDashboard.putNumber("Wrist Position: ", Robot.intake.getWristEncoder());
        SmartDashboard.putNumber("Elbow Position: ", Robot.intake.getElbowEncoder());
        SmartDashboard.putNumber("Intake State: ", Robot.intake.intakeState);
        SmartDashboard.putBoolean("Piece Acquired: ", Robot.intake.getIntakeCurrent() > 20);

        Robot.intake.intakeLog();
    }
}
