package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.CANID;
import frc.robot.util.Logger;

public class Intake extends SubsystemBase {
    private CANSparkMax elbowRotator;
    private CANSparkMax wristRotator;
    private CANSparkMax intake;
    private CANSparkMax UBI_1;
    private CANSparkMax UBI_2;
    
    private RelativeEncoder elbowEncoder;
    private RelativeEncoder wristEncoder;

    private Timer intakeTimer;

    public SparkPIDController elbowPIDF;
    public SparkPIDController wristPIDF;

    public double wristSetPoint;
    public double elbowSetPoint;
    public int intakeState = 0;
    public boolean intaking = false;
    public boolean fcControlElbow;
    public boolean fcControlWrist;

    public Intake() {
        elbowRotator = new CANSparkMax(CANID.ELBOW_RIGHT, MotorType.kBrushless);
        wristRotator = new CANSparkMax(CANID.WRIST, MotorType.kBrushless);
        intake = new CANSparkMax(CANID.INTAKE, MotorType.kBrushless);
        UBI_1 = new CANSparkMax(CANID.UBI_1, MotorType.kBrushless);
        UBI_2 = new CANSparkMax(CANID.UBI_2, MotorType.kBrushless);

        elbowRotator.setIdleMode(IdleMode.kBrake);
        wristRotator.setIdleMode(IdleMode.kBrake);
        intake.setIdleMode(IdleMode.kBrake);
        UBI_1.setIdleMode(IdleMode.kBrake);
        UBI_2.setIdleMode(IdleMode.kBrake);

        elbowEncoder = elbowRotator.getEncoder();
        wristEncoder = wristRotator.getEncoder();

        elbowPIDF = elbowRotator.getPIDController();
        wristPIDF = wristRotator.getPIDController();

        elbowRotator.enableSoftLimit(SoftLimitDirection.kForward, true);
        elbowRotator.enableSoftLimit(SoftLimitDirection.kReverse, true);
    
        wristRotator.enableSoftLimit(SoftLimitDirection.kForward, true);
        wristRotator.enableSoftLimit(SoftLimitDirection.kReverse, true);

        elbowRotator.setSoftLimit(SoftLimitDirection.kForward, 160);
        elbowRotator.setSoftLimit(SoftLimitDirection.kReverse, 0);
        wristRotator.setSoftLimit(SoftLimitDirection.kForward, 170);
        wristRotator.setSoftLimit(SoftLimitDirection.kReverse, -80);

        elbowRotator.setSmartCurrentLimit(IntakeConstants.kElbowCurrentLimit);
        wristRotator.setSmartCurrentLimit(IntakeConstants.kWristCurrentLimit);
        intake.setSmartCurrentLimit(IntakeConstants.kIntakeCurrentLimit);
        UBI_1.setSmartCurrentLimit(80);
        UBI_2.setSmartCurrentLimit(80);
        UBI_1.setSmartCurrentLimit(80);
        UBI_2.setSmartCurrentLimit(80);

        elbowEncoder.setPositionConversionFactor(IntakeConstants.kElbowEncoderDistancePerPulse);
        wristEncoder.setPositionConversionFactor(IntakeConstants.kWristEncoderDistancePerPulse);

        elbowPIDF.setP(IntakeConstants.kPElbow);
        wristPIDF.setP(IntakeConstants.kPWrist);

        elbowPIDF.setOutputRange(-0.85, 0.85);
        wristPIDF.setOutputRange(-1, 1);

        intakeTimer = new Timer();

        elbowRotator.burnFlash();
        wristRotator.burnFlash();
        intake.burnFlash();
        UBI_1.burnFlash();
        UBI_2.burnFlash();
    }

    public void setIntakeCurrentLimit(int lim) {
        intake.setSmartCurrentLimit(lim);
    }

    public double getTimer() {
        return intakeTimer.get();
    }

    public void resetTimer() {
        intakeTimer.reset();
	    intakeTimer.start();
    }

    public double getElbowEncoder() {
        return elbowEncoder.getPosition();
    }

    public double getWristEncoder() {
        return wristEncoder.getPosition();
    }

    public double getIntakeCurrent() {
        return intake.getOutputCurrent();
    }
    
    public void setElbowPosition(double setPoint) {
        elbowPIDF.setReference(setPoint, CANSparkBase.ControlType.kPosition);
    }

	public void setWristPosition(double setPoint) {
        wristPIDF.setReference(setPoint, CANSparkBase.ControlType.kPosition);
    }

    public void setIntakeVoltage(double upper, double lower) {
        intake.setVoltage(upper);
        UBI_1.setVoltage(lower);
        UBI_2.setVoltage(-lower);
    }

    public void intakeLog(){
        Logger.info("ELBOW", Double.toString(getElbowEncoder()) + " Actual Degrees -> " + Double.toString(elbowSetPoint) + " Target Degrees");
        Logger.info("WRIST", Double.toString(getWristEncoder()) + " Actual Degrees -> " + Double.toString(wristSetPoint) + " Target Degrees");
        if(elbowRotator.getFaults()!=0){Logger.warn("ELBOW: " + Short.toString(elbowRotator.getFaults()));}
        if(wristRotator.getFaults()!=0){Logger.warn("WRIST: " + Short.toString(wristRotator.getFaults()));}
        if(intake.getFaults()!=0){Logger.warn("INTKL: " + Short.toString(intake.getFaults()));}
    }
}
