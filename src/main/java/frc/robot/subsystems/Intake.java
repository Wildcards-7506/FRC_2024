package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
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
    private CANSparkMax intake;
    
    private RelativeEncoder elbowEncoder;
    private RelativeEncoder wristEncoder;
    private RelativeEncoder intakeEncoder;

    public SparkPIDController elbowPIDF;
    public SparkPIDController wristPIDF;

    public double wristSetPoint  = IntakeConstants.kWristStowed;
    public double elbowSetPoint  = IntakeConstants.kElbowStowed;
    public int intakeState = 4; //<- CHANGE THIS TO ZERO BEFORE COMPS
    public boolean running = false;
    public boolean pieceAcquired = false;
    public boolean fcControlElbow;
    public boolean fcControlWrist;

    public Intake() {
        elbowRotatorLeader = new CANSparkMax(CANID.ELBOW_RIGHT, MotorType.kBrushless);
        elbowRotatorFollower = new CANSparkMax(CANID.ELBOW_LEFT, MotorType.kBrushless);
        wristRotator = new CANSparkMax(CANID.WRIST, MotorType.kBrushless);
        intake = new CANSparkMax(CANID.INTAKE, MotorType.kBrushless);

        elbowRotatorLeader.setIdleMode(IdleMode.kBrake);
        elbowRotatorFollower.setIdleMode(IdleMode.kBrake);
        wristRotator.setIdleMode(IdleMode.kBrake);
        intake.setIdleMode(IdleMode.kCoast);

        elbowEncoder = elbowRotatorLeader.getEncoder();
        wristEncoder = wristRotator.getEncoder();
        intakeEncoder = intake.getEncoder();

        elbowPIDF = elbowRotatorLeader.getPIDController();
        wristPIDF = wristRotator.getPIDController();

        intake.setInverted(true);
        wristRotator.setInverted(true);
        elbowRotatorFollower.follow(elbowRotatorLeader, true);

        elbowRotatorLeader.enableSoftLimit(SoftLimitDirection.kForward, true);
        elbowRotatorLeader.enableSoftLimit(SoftLimitDirection.kReverse, true);
        elbowRotatorFollower.enableSoftLimit(SoftLimitDirection.kForward, true);
        elbowRotatorFollower.enableSoftLimit(SoftLimitDirection.kReverse, true);
    
        wristRotator.enableSoftLimit(SoftLimitDirection.kForward, true);
        wristRotator.enableSoftLimit(SoftLimitDirection.kReverse, true);

        intake.enableSoftLimit(SoftLimitDirection.kForward, false);
        intake.enableSoftLimit(SoftLimitDirection.kReverse, false);

        // Will need to test these angle parameters when testing
        elbowRotatorLeader.setSoftLimit(SoftLimitDirection.kForward, 160);
        elbowRotatorLeader.setSoftLimit(SoftLimitDirection.kReverse, 0);
        elbowRotatorFollower.setSoftLimit(SoftLimitDirection.kForward, 0);
        elbowRotatorFollower.setSoftLimit(SoftLimitDirection.kReverse, -160);
        wristRotator.setSoftLimit(SoftLimitDirection.kForward, 190);
        wristRotator.setSoftLimit(SoftLimitDirection.kReverse, -90);

        elbowRotatorLeader.setSmartCurrentLimit(IntakeConstants.kElbowCurrentLimit);
        elbowRotatorFollower.setSmartCurrentLimit(IntakeConstants.kElbowCurrentLimit);
        wristRotator.setSmartCurrentLimit(IntakeConstants.kWristCurrentLimit);
        intake.setSmartCurrentLimit(IntakeConstants.kIntakeCurrentLimit);

        elbowEncoder.setPositionConversionFactor(IntakeConstants.kElbowEncoderDistancePerPulse);
        wristEncoder.setPositionConversionFactor(IntakeConstants.kWristEncoderDistancePerPulse);

        elbowPIDF.setP(IntakeConstants.kPElbow);
        wristPIDF.setP(IntakeConstants.kPWrist);

        elbowPIDF.setOutputRange(-1, 1);
        wristPIDF.setOutputRange(-1, 1);

        elbowRotatorLeader.burnFlash();
        elbowRotatorFollower.burnFlash();
        wristRotator.burnFlash();
        intake.burnFlash();
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

    public double getIntakeSpeed() {
        return intakeEncoder.getVelocity();
    }
    
    public void setElbowPosition(double setPoint) {
        elbowPIDF.setReference(setPoint, CANSparkBase.ControlType.kPosition);
    }

	public void setWristPosition(double setPoint) {
        wristPIDF.setReference(setPoint, CANSparkBase.ControlType.kPosition);
    }

    public void setIntakeVoltage(double setPoint) {
        intake.setVoltage(setPoint);
    }

    public void intakeLog(){
        Logger.info("ELBOW", Double.toString(getElbowEncoder()) + " Actual Degrees -> " + Double.toString(elbowSetPoint) + " Target Degrees");
        Logger.info("WRIST", Double.toString(getWristEncoder()) + " Actual Degrees -> " + Double.toString(wristSetPoint) + " Target Degrees");
        if(elbowRotatorLeader.getFaults()!=0){Logger.warn("ELBWL: " + Short.toString(elbowRotatorLeader.getFaults()));}
        if(elbowRotatorFollower.getFaults()!=0){Logger.warn("ELBWF: " + Short.toString(elbowRotatorFollower.getFaults()));}
        if(wristRotator.getFaults()!=0){Logger.warn("WRIST: " + Short.toString(wristRotator.getFaults()));}
        if(intake.getFaults()!=0){Logger.warn("INTKL: " + Short.toString(intake.getFaults()));}
    }
}
