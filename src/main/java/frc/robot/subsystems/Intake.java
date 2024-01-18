package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.CANID;
import frc.robot.util.Logger;

public class Intake extends SubsystemBase {
    private CANSparkMax elbowRotatorLeader;
    private CANSparkMax elbowRotatorFollower;
    private CANSparkMax wristRotator;
    private CANSparkMax intakeLeader;
    private CANSparkMax intakeFollower;
    
    private RelativeEncoder elbowEncoder;
    private RelativeEncoder wristEncoder;
    private RelativeEncoder intakeEncoder;

    private SparkPIDController elbowPID;
    private SparkPIDController wristPID;

    public double wristSetPoint = 0;
    public double elbowSetPoint = 0;
    public int intakeState = 0;
    public boolean running = false;
    public boolean pieceAcquired = false;

    public Intake() {
        elbowRotatorLeader = new CANSparkMax(CANID.ELBOW_LEFT, MotorType.kBrushless);
        elbowRotatorFollower = new CANSparkMax(CANID.ELBOW_RIGHT, MotorType.kBrushless);
        wristRotator = new CANSparkMax(CANID.WRIST, MotorType.kBrushless);
        intakeLeader = new CANSparkMax(CANID.INTAKE_LEFT, MotorType.kBrushless);
        intakeFollower = new CANSparkMax(CANID.INTAKE_RIGHT, MotorType.kBrushless);

        elbowEncoder = elbowRotatorLeader.getEncoder();
        wristEncoder = wristRotator.getEncoder();
        intakeEncoder = intakeLeader.getEncoder();

        elbowPID = elbowRotatorLeader.getPIDController();
        wristPID = wristRotator.getPIDController();

        elbowRotatorFollower.follow(elbowRotatorLeader, true);
        intakeFollower.follow(intakeLeader, true);

        elbowRotatorLeader.enableSoftLimit(SoftLimitDirection.kForward, true);
        elbowRotatorLeader.enableSoftLimit(SoftLimitDirection.kReverse, true);
        elbowRotatorFollower.enableSoftLimit(SoftLimitDirection.kForward, true);
        elbowRotatorFollower.enableSoftLimit(SoftLimitDirection.kReverse, true);
    
        wristRotator.enableSoftLimit(SoftLimitDirection.kForward, true);
        wristRotator.enableSoftLimit(SoftLimitDirection.kReverse, true);

        intakeLeader.enableSoftLimit(SoftLimitDirection.kForward, false);
        intakeLeader.enableSoftLimit(SoftLimitDirection.kReverse, false);
        intakeFollower.enableSoftLimit(SoftLimitDirection.kForward, false);
        intakeFollower.enableSoftLimit(SoftLimitDirection.kReverse, false);

        // Will need to test these angle parameters when testing
        elbowRotatorLeader.setSoftLimit(SoftLimitDirection.kForward, 160);
        elbowRotatorLeader.setSoftLimit(SoftLimitDirection.kReverse, 0);
        elbowRotatorFollower.setSoftLimit(SoftLimitDirection.kForward, 0);
        elbowRotatorFollower.setSoftLimit(SoftLimitDirection.kReverse, 160);
        wristRotator.setSoftLimit(SoftLimitDirection.kForward, 190);
        wristRotator.setSoftLimit(SoftLimitDirection.kReverse, 0);

        elbowRotatorLeader.setSmartCurrentLimit(IntakeConstants.kElbowCurrentLimit);
        elbowRotatorFollower.setSmartCurrentLimit(IntakeConstants.kElbowCurrentLimit);
        wristRotator.setSmartCurrentLimit(IntakeConstants.kWristCurrentLimit);
        intakeLeader.setSmartCurrentLimit(IntakeConstants.kIntakeCurrentLimit);
        intakeFollower.setSmartCurrentLimit(IntakeConstants.kIntakeCurrentLimit);

        elbowEncoder.setPositionConversionFactor(IntakeConstants.kElbowEncoderDistancePerPulse);
        wristEncoder.setPositionConversionFactor(IntakeConstants.kWristEncoderDistancePerPulse);

        elbowPID.setP(IntakeConstants.kElbowKP);
        wristPID.setP(IntakeConstants.kWristKP);

        elbowPID.setOutputRange(-1, 1);
        wristPID.setOutputRange(-1, 1);

        elbowRotatorLeader.burnFlash();
        elbowRotatorFollower.burnFlash();
        wristRotator.burnFlash();
        intakeLeader.burnFlash();
        intakeFollower.burnFlash();
    }

    public double getElbowEncoder() {
        return elbowEncoder.getPosition();
    }

    public double getWristEncoder() {
        return wristEncoder.getPosition();
    }

    public double getIntakeCurrent() {
        return intakeLeader.getOutputCurrent();
    }

    public double getIntakeSpeed() {
        return intakeEncoder.getVelocity();
    }
    
    public void setElbowPosition(double setPoint) {
        elbowPID.setReference(setPoint, CANSparkBase.ControlType.kPosition);
    }

	public void setWristPosition(double setPoint) {
        wristPID.setReference(setPoint, CANSparkBase.ControlType.kPosition);
    }

    public void setIntakeVoltage(double setPoint) {
        intakeLeader.setVoltage(setPoint);
    }

    public void intakeLog(){
        Logger.info("ELBOW", Double.toString(getElbowEncoder()) + " Actual Degrees -> " + Double.toString(elbowSetPoint) + " Target Degrees");
        Logger.info("WRIST", Double.toString(getWristEncoder()) + " Actual Degrees -> " + Double.toString(wristSetPoint) + " Target Degrees");
        if(elbowRotatorLeader.getFaults()!=0){Logger.warn("ELBWL: " + Short.toString(elbowRotatorLeader.getFaults()));}
        if(elbowRotatorFollower.getFaults()!=0){Logger.warn("ELBWF: " + Short.toString(elbowRotatorFollower.getFaults()));}
        if(wristRotator.getFaults()!=0){Logger.warn("WRIST: " + Short.toString(wristRotator.getFaults()));}
        if(intakeLeader.getFaults()!=0){Logger.warn("INTKL: " + Short.toString(intakeLeader.getFaults()));}
        if(intakeFollower.getFaults()!=0){Logger.warn("INTKF: " + Short.toString(intakeFollower.getFaults()));}
    }
}
