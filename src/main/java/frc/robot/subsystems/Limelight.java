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
import frc.robot.Constants;
import frc.robot.Constants.CANID;

public class Limelight extends SubsystemBase{
    public double distance;
    public double offset;

    public NetworkTable table;
    private NetworkTableEntry tx;
    private NetworkTableEntry ta;
    private NetworkTableEntry tv;
    private NetworkTableEntry ty;

    private SparkPIDController llPIDController;
    private RelativeEncoder llEncoder;
    private CANSparkMax llRotator;

    public Limelight () {
        llRotator = new CANSparkMax(CANID.LL_ROTATOR, MotorType.kBrushless);

        llEncoder = llRotator.getEncoder();

        llPIDController = llRotator.getPIDController();
        llPIDController.setOutputRange(-1, 1);
        llRotator.setSmartCurrentLimit(Constants.limelightRotatorConstants.kRotateCurrentLimit);
        llPIDController.setP(Constants.limelightRotatorConstants.kRotatorKP);

        llRotator.burnFlash();
    }

    public void setLLRotatorPosition(double kPosition) {
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

}
