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
        if(PlayerConfigs.intake){
            Robot.intake.intakeState = 1;
            Robot.intake.pieceAcquired = false;
            Robot.intake.running = false;
        } else if(PlayerConfigs.amp){
            Robot.intake.intakeState = 2;
        } else if(!Robot.shooter.shootingMode && (PlayerConfigs.climberDown || PlayerConfigs.climberUp)){
            //Only climber control can initiate trap position, no manual triggering allowed
            //To activate trap, disable shooter and run climber down
            Robot.intake.intakeState = 3;
        } else if(PlayerConfigs.fcEnable){
            //When fine control is enabled, grab current positions to hold
            Robot.intake.intakeState = 4;
            Robot.intake.fcControlElbow = false;
            Robot.intake.fcControlWrist = false;
            Robot.intake.elbowSetPoint = Robot.intake.getElbowEncoder();
            Robot.intake.wristSetPoint = Robot.intake.getWristEncoder();
        } else if(PlayerConfigs.stow){
            Robot.intake.intakeState = 0;
        }

        //Ground State
        if (Robot.intake.intakeState == 1) {
            //Hold intake to stowed or contrained position to avoid damage or extension rule until low enough to open to ground position
            if (Robot.intake.getElbowEncoder() < IntakeConstants.kElbowDownConstraint) {
                Robot.intake.wristSetPoint = IntakeConstants.kWristGround;
            } else if (Robot.intake.getElbowEncoder() < 140){
                Robot.intake.wristSetPoint = IntakeConstants.kWristConstraint;
            } else {
                Robot.intake.wristSetPoint = IntakeConstants.kWristStowed;
            }
            
            // Only move to ground when wrist is constrained to not break extension rule
            // Prevent fluttering as wrist comes out to ground position by locking elbow setpoint to ground once triggered
            if (Robot.intake.getWristEncoder() < -60 || Robot.intake.elbowSetPoint == IntakeConstants.kElbowGround){
                Robot.intake.elbowSetPoint = IntakeConstants.kElbowGround;
            } else {
                Robot.intake.elbowSetPoint = IntakeConstants.kElbowUpConstraint;
            }  
        //Amp State
        } else if (Robot.intake.intakeState == 2) {
            //Wrist stays in safe position (stowed or constrained) until elbow is in target position
            if (Robot.intake.getElbowEncoder() > 140) {
                Robot.intake.wristSetPoint = IntakeConstants.kWristStowed;
            } else if (Robot.intake.getElbowEncoder() < 115){
                Robot.intake.wristSetPoint = IntakeConstants.kWristConstraint;
            } else {
                Robot.intake.wristSetPoint = IntakeConstants.kWristAmp;
            }

            Robot.intake.elbowSetPoint = IntakeConstants.kElbowAmp;
        //Trap State
        } else if (Robot.intake.intakeState == 3) {
            //Elbow goes to pressure setting if actively climbing. Once high enough to rest on static structural member, elbow moves to trap scoring position
            if(Robot.climbers.getClimberEncoder() > ClimberConstants.scoringHeight){
                Robot.intake.elbowSetPoint = IntakeConstants.kElbowTrapScoring;
            } else {
                Robot.intake.elbowSetPoint = IntakeConstants.kElbowTrapPressure;
            }
            
            //Intake stays stowed and out of danger until high enough to rest on static structural member
            if (Robot.climbers.getClimberEncoder() < ClimberConstants.scoringHeight) {
                Robot.intake.wristSetPoint = IntakeConstants.kWristTrap;
            } else {
                Robot.intake.wristSetPoint = IntakeConstants.kWristStowed;
            }
        //Fine Control
        } else if(Robot.intake.intakeState == 4) {
            //If joysticks are outside of deadband, move individual sections up or down
            //If joysticks are no longer outside of deadband and control is still active, disable control and grab the current position to hold
            //Grabbing position once eliminates gradual sinking due to gravity
            if(Math.abs(PlayerConfigs.fcElbow) > 0.25){
                Robot.intake.fcControlElbow = true;
                Robot.intake.elbowSetPoint = Robot.intake.getElbowEncoder() - IntakeConstants.kElbowManualOffset * PlayerConfigs.fcElbow; 
            } else if(Robot.intake.fcControlElbow){
                Robot.intake.fcControlElbow = false;
                Robot.intake.elbowSetPoint = Robot.intake.getElbowEncoder();
            }

            if(Math.abs(PlayerConfigs.fcWrist) > 0.25){
                Robot.intake.fcControlWrist = true;
                Robot.intake.wristSetPoint = Robot.intake.getWristEncoder() - IntakeConstants.kWristManualOffset * PlayerConfigs.fcWrist;
            } else if(Robot.intake.fcControlWrist){
                Robot.intake.fcControlWrist = false;
                Robot.intake.wristSetPoint = Robot.intake.getWristEncoder();
            }
            
        //Stow and Shoot
        } else {
            //Stow wrist if inside extension limit, otherwise hold in constraint position. Only move to shooting position if safe to do so
            if ((Math.abs(IntakeConstants.kElbowStowed - Robot.intake.getElbowEncoder()) < 10) && PlayerConfigs.armScoringMechanism) {
                Robot.intake.wristSetPoint = IntakeConstants.kWristShooting;
            } else if(Robot.intake.getElbowEncoder() > 80){
                Robot.intake.wristSetPoint = IntakeConstants.kWristStowed;
            } else {
                Robot.intake.wristSetPoint = IntakeConstants.kWristConstraint;
            }
            
            //Only move elbow all the way to stow position if wrist is already in stowed position
            if (Robot.intake.getWristEncoder() < IntakeConstants.kWristShooting - 5 && !PlayerConfigs.armScoringMechanism) {
                Robot.intake.elbowSetPoint = IntakeConstants.kElbowUpConstraint;
            } else {
                Robot.intake.elbowSetPoint = IntakeConstants.kElbowStowed;
            }
        }

        //Once setpoints have been chosen, pass to controllers
        Robot.intake.setElbowPosition(Robot.intake.elbowSetPoint);
        Robot.intake.setWristPosition(Robot.intake.wristSetPoint);

        //Run the intake if a piece has not been acquired in ground position or fire button is pressed
        if ((Robot.intake.intakeState == 1 && !Robot.intake.pieceAcquired) || (PlayerConfigs.fire)) {
            Robot.intake.setIntakeVoltage(12);
            //A game piece is considered captured if the intake has passed 200 rpm (gets rid of stall current spike when starting) and a current spike is detected.
            //Tune current trip point to avoid false positives but also intake the piece securely
            Robot.intake.running = Robot.intake.getIntakeSpeed() > 200 ? true : Robot.intake.running;
            Robot.intake.pieceAcquired = (Robot.intake.running && Robot.intake.getIntakeCurrent() > 20) ? true : false;
        } else {
            Robot.intake.setIntakeVoltage(0);
            Robot.intake.running = false;
        }

        SmartDashboard.putNumber("Wrist Setpoint: ", Robot.intake.wristSetPoint);
        SmartDashboard.putNumber("Elbow Setpoint: ", Robot.intake.elbowSetPoint);
        SmartDashboard.putNumber("Wrist Position: ", Robot.intake.getWristEncoder());
        SmartDashboard.putNumber("Elbow Position: ", Robot.intake.getElbowEncoder());
        SmartDashboard.putNumber("Intake State: ",Robot.intake.intakeState);
        SmartDashboard.putBoolean("Piece Acquired: ", Robot.intake.pieceAcquired);
        SmartDashboard.putNumber("error", Math.abs(Robot.intake.elbowSetPoint - Robot.intake.getElbowEncoder()));

        Robot.intake.intakeLog();
    }
}
