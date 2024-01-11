package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ExampleArmConstants;
import frc.robot.util.Logger;

public class ExampleArm extends SubsystemBase {
    private CANSparkMax armMotor;
    private CANSparkMax intakeMotor;
    
    private RelativeEncoder armEncoder;

    public SparkPIDController armPID;

    public ExampleArm(int arm, int intake) {

        intakeMotor = new CANSparkMax(intake, MotorType.kBrushless);
        armMotor = new CANSparkMax(arm, MotorType.kBrushless);

        armEncoder = armMotor.getEncoder();

        armEncoder.setPositionConversionFactor(ExampleArmConstants.kArmEncoderDistancePerPulse);
    
        armMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        intakeMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
        intakeMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);

        armMotor.setSmartCurrentLimit(ExampleArmConstants.kArmCurrentLimit);
        intakeMotor.setSmartCurrentLimit(ExampleArmConstants.kIntakeCurrentLimit);

        armMotor.setSoftLimit(SoftLimitDirection.kForward, 330);
        armMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);

        armPID = armMotor.getPIDController();

        armPID.setP(ExampleArmConstants.kArmKP);

        armPID.setOutputRange(-1, 1);

        armMotor.burnFlash();
        intakeMotor.burnFlash();
    }

    public double getArmEncoder() {
        return armEncoder.getPosition();
    }

    public double getIntakeCurrent(){
        return intakeMotor.getOutputCurrent();
    }

    public void setArm(double setPoint) {
        double arbFF = 0 * Math.cos(Math.toRadians(getArmEncoder() - ExampleArmConstants.armHorizontalOffset));
        armPID.setReference(setPoint, ControlType.kPosition, 0, arbFF);
        SmartDashboard.putNumber("Arm Setpoint", setPoint);
    }

    public void setIntake (double setPoint) {
        intakeMotor.setVoltage(setPoint);
    }

    public void errorCheck(){
        if(intakeMotor.getFaults()!=0){Logger.warn("INTKE: " + Short.toString(intakeMotor.getFaults()));}
        if(armMotor.getFaults()!=0){Logger.warn("ARM: " + Short.toString(armMotor.getFaults()));}
    }
}