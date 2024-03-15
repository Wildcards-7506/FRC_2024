package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase{
    public double distance;
    public double offset;

    public NetworkTable table;
    private NetworkTableEntry tx;
    private NetworkTableEntry tid;

    public Limelight () {}

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
}
