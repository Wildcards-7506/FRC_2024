package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Robot;
import frc.robot.ControlConfigs.PlayerConfigs;

public class IntakeTeleopCommand extends Command{
    
    public IntakeTeleopCommand(){
        addRequirements(Robot.intake);
    }
    
    @Override
    public void execute (){
        if(PlayerConfigs.intake){
            Robot.intake.intakeState = 1;
            Robot.intake.pieceAcquired = false;
        } else if(PlayerConfigs.amp){
            Robot.intake.intakeState = 2;
        } else if(PlayerConfigs.trap){
            Robot.intake.intakeState = 3;
        } else if(PlayerConfigs.stow){
            Robot.intake.intakeState = 0;
        }


        if (Robot.intake.getWristEncoder() > 170) {
            if (Robot.intake.intakeState == 1) {
                Robot.intake.elbowSetPoint = IntakeConstants.kElbowGround;
            } else if (Robot.intake.intakeState == 2) {
                Robot.intake.elbowSetPoint = IntakeConstants.kElbowAmp;
            } else if (Robot.intake.intakeState == 3) {
                Robot.intake.elbowSetPoint = IntakeConstants.kElbowTrap;
            } else {
                Robot.intake.elbowSetPoint = IntakeConstants.kElbowStowed;
            }
        } else {
            Robot.intake.elbowSetPoint = Robot.intake.getElbowEncoder();
        }

        if (Robot.intake.intakeState == 1) {
            if (Math.abs(IntakeConstants.kElbowGround - Robot.intake.getElbowEncoder()) < 10) {
                Robot.intake.wristSetPoint = IntakeConstants.kWristGround;
            } else {
                Robot.intake.wristSetPoint = IntakeConstants.kWristStowed;
            }
        } else if (Robot.intake.intakeState == 2){
            if (Math.abs(IntakeConstants.kElbowAmp  - Robot.intake.getElbowEncoder()) < 10) {
                Robot.intake.wristSetPoint = IntakeConstants.kWristAmp;
            } else {
                Robot.intake.wristSetPoint = IntakeConstants.kWristStowed;
            }
        } else if (Robot.intake.intakeState == 3) {
            if (Math.abs(IntakeConstants.kElbowTrap  - Robot.intake.getElbowEncoder()) < 10 && PlayerConfigs.armScoringMechanism) {
                Robot.intake.wristSetPoint = IntakeConstants.kWristTrap;
            } else {
                Robot.intake.wristSetPoint = IntakeConstants.kWristStowed;
            }
        } else if (PlayerConfigs.armScoringMechanism) {
            if (Math.abs(IntakeConstants.kElbowStowed  - Robot.intake.getElbowEncoder()) < 10) {
                Robot.intake.wristSetPoint = IntakeConstants.kWristShooting;
            } else {
                Robot.intake.wristSetPoint = IntakeConstants.kWristStowed;
            }
        } else {
            Robot.intake.wristSetPoint = IntakeConstants.kWristStowed;
        }

        Robot.intake.setWristPosition(Robot.intake.wristSetPoint);
        Robot.intake.setElbowPosition(Robot.intake.elbowSetPoint);

        if ((Robot.intake.intakeState == 1 &! Robot.intake.pieceAcquired) || ((Robot.intake.intakeState == 0 || Robot.intake.intakeState == 3) && PlayerConfigs.fire)) {
            Robot.intake.setIntakeVoltage(12);
            Robot.intake.running = Robot.intake.getIntakeSpeed() > 200 ? true : false;
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
