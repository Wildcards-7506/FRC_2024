package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANID;
import frc.robot.Constants.LimelightConstants;
import frc.robot.util.Logger;

public class Limelight extends SubsystemBase{
    public double distance;
    public double offset;

    public NetworkTable table;
    private NetworkTableEntry tx;
    private NetworkTableEntry tid;

    private SparkPIDController limelightPIDController;
    private RelativeEncoder limelightEncoder;
    private CANSparkMax limelightRotator;

    public Limelight () {
        limelightRotator = new CANSparkMax(CANID.LIMELIGHT, MotorType.kBrushless);

        limelightEncoder = limelightRotator.getEncoder();
        limelightEncoder.setPositionConversionFactor(LimelightConstants.kRotatorEncoderDistancePerPulse);

        limelightPIDController = limelightRotator.getPIDController();
        limelightPIDController.setOutputRange(-1, 1);
        limelightRotator.setSmartCurrentLimit(LimelightConstants.kRotateCurrentLimit);
        limelightPIDController.setP(LimelightConstants.kRotatorKP);

        limelightRotator.setIdleMode(IdleMode.kBrake);

        limelightRotator.burnFlash();
    }

    public void setLimelightPosition(double kPosition) {
        limelightPIDController.setReference(kPosition, ControlType.kPosition);
    }

    public double getPos() {
        return limelightEncoder.getPosition();
    }

    //The following five methods retrieve and make data available from the Limelight Network Table
    public void updateData() {
        // update table, then update from updated table
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        tid = table.getEntry("tid");
    }

    public double getTX() {
        updateData();
        return tx.getDouble(0.0);
    }

    public double getID() {
        updateData();
        return tid.getDouble(0.0);
    }

    public void setPipeline(double pipeline) {
        table.getEntry("pipeline").setNumber(pipeline);
    }

    public void limelightLog(){
        Logger.info("LMLGT", Double.toString(getPos()) + " Degrees");
        if(limelightRotator.getFaults()!=0){Logger.warn("LMLGT: " + Short.toString(limelightRotator.getFaults()));}
    }

}
