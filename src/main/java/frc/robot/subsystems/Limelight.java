package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANID;
import frc.robot.Constants.LimelightConstants;

public class Limelight extends SubsystemBase{
    public double distance;
    public double offset;

    public NetworkTable table;
    private NetworkTableEntry tx;
    private NetworkTableEntry ta;
    private NetworkTableEntry tv;
    private NetworkTableEntry ty;
    private NetworkTableEntry tid;

    private SparkPIDController llPIDController;
    private RelativeEncoder llEncoder;
    private CANSparkMax llRotator;

    public Limelight () {
        llRotator = new CANSparkMax(CANID.LIMELIGHT, MotorType.kBrushless);

        llEncoder = llRotator.getEncoder();

        llPIDController = llRotator.getPIDController();
        llPIDController.setOutputRange(-1, 1);
        llRotator.setSmartCurrentLimit(LimelightConstants.kRotateCurrentLimit);
        llPIDController.setP(LimelightConstants.kRotatorKP);

        llRotator.burnFlash();
    }

    public void setLimelightPosition(double kPosition) {
        llPIDController.setReference(kPosition, ControlType.kPosition);
    }

    public double getPos() {
        return llEncoder.getPosition();
    }

    //The following five methods retrieve and make data available from the Limelight Network Table
    public void updateData() {
        // update table, then update from updated table
        table = NetworkTableInstance.getDefault().getTable("limelight");
        ta = table.getEntry("ta");
        tv = table.getEntry("tv");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        tid = table.getEntry("tid");
    }

    public double getTX() {
        updateData();
        return tx.getDouble(0.0);
    }

    public double getTY() {
        updateData();
        return ty.getDouble(0.0);
    }

    public double getTA() {
        updateData();
        return ta.getDouble(0.0);
    }

    public double getTV() {
        updateData();
        return tv.getDouble(0.0);
    }

    public double getID() {
        updateData();
        return tid.getDouble(0.0);
    }

}
