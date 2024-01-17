package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.ControlConfigs.PlayerConfigs;

public class IntakeTeleopCommand extends Command{
    
    public IntakeTeleopCommand(){
        addRequirements(Robot.intake);
    }
    
    @Override
    public void execute (){
        if (Robot.intake.getWristEncoder() > 170) {
            if (PlayerConfigs.intake) {
                Robot.intake.elbowSetPoint = Constants.IntakeConstants.kElbowGround;
            } else if (PlayerConfigs.amp) {
                Robot.intake.elbowSetPoint = Constants.IntakeConstants.kElbowAmp;
            } else if (PlayerConfigs.trap) {
                Robot.intake.elbowSetPoint = Constants.IntakeConstants.kElbowTrap;
            } else {
                Robot.intake.elbowSetPoint = Constants.IntakeConstants.kElbowStowed;
            }
        } else {
            Robot.intake.elbowSetPoint = Robot.intake.getElbowEncoder();
        }

        if (PlayerConfigs.intake) {
            if (Math.abs(Robot.intake.elbowSetPoint - Robot.intake.getElbowEncoder()) < 10) {
                Robot.intake.wristSetPoint = Constants.IntakeConstants.kWristGround;
            } else {
                Robot.intake.wristSetPoint = Constants.IntakeConstants.kWristStowed;
            }
        } else if (PlayerConfigs.amp){
            if (Math.abs(Robot.intake.elbowSetPoint - Robot.intake.getElbowEncoder()) < 10) {
                Robot.intake.wristSetPoint = Constants.IntakeConstants.kWristAmp;
            } else {
                Robot.intake.wristSetPoint = Constants.IntakeConstants.kWristStowed;
            }
        } else if (PlayerConfigs.trap) {
            if (Math.abs(Robot.intake.elbowSetPoint - Robot.intake.getElbowEncoder()) < 10) {
                Robot.intake.wristSetPoint = Constants.IntakeConstants.kWristTrap;
            } else {
                Robot.intake.wristSetPoint = Constants.IntakeConstants.kWristStowed;
            }
        } else if (PlayerConfigs.shooterArmed) {
            if (Math.abs(Robot.intake.elbowSetPoint - Robot.intake.getElbowEncoder()) < 10) {
                Robot.intake.wristSetPoint = Constants.IntakeConstants.kWristShooting;
            } else {
                Robot.intake.wristSetPoint = Constants.IntakeConstants.kWristStowed;
            }
        } else {
            Robot.intake.wristSetPoint = Constants.IntakeConstants.kWristStowed;
        }

        Robot.intake.setWristPosition(Robot.intake.wristSetPoint);
        Robot.intake.setElbowPosition(Robot.intake.elbowSetPoint);

        if (PlayerConfigs.intake && PlayerConfigs.release) {
            Robot.intake.setIntakeVoltage(12);
        } else {
            Robot.intake.setIntakeVoltage(0);
        }

        SmartDashboard.putNumber("Wrist Setpoint: ", Robot.intake.wristSetPoint);
        SmartDashboard.putNumber("Elbow Setpoint: ", Robot.intake.elbowSetPoint);
    }
}
