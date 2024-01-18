package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Robot;
import frc.robot.Constants.CANID;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.LimelightConstants;
import frc.robot.util.Logger;

public class Limelight implements AutoCloseable{
    // The arm gearbox represents a gearbox containing two Vex 775pro motors.
    private final DCMotor m_limelightGearbox = DCMotor.getNeo550(1);

    // Standard classes for controlling our arm
    private final PIDController m_limelightController = new PIDController(LimelightConstants.kRotatorKP, 0, 0.5);
    public final Encoder m_limelightEncoder =
        new Encoder(8, 9);
    private final PWMSparkMax m_limelightMotor = new PWMSparkMax(CANID.LIMELIGHT);

    // Simulation classes help us simulate what's going on, including gravity.
    // This arm sim represents an arm that can travel from -75 degrees (rotated down front)
    // to 255 degrees (rotated down in the back).
    private final SingleJointedArmSim m_limelightSim =
        new SingleJointedArmSim(
            m_limelightGearbox,
            1.0/15,
            SingleJointedArmSim.estimateMOI(0.1, 0.1),
            0.1,
            -7*Math.PI/4,
            3*Math.PI/4,
            false,
            0,
            VecBuilder.fill(0.01/(2*Math.PI)) // Add noise with a std-dev of 1 tick
            );
    private final EncoderSim m_limelightEncoderSim = new EncoderSim(m_limelightEncoder);

    private final Mechanism2d m_limelightMech2d = new Mechanism2d(20, 20);
  private final MechanismRoot2d m_limelightPivot = m_limelightMech2d.getRoot("limelightPivot", 10, 10);
  private final MechanismLigament2d m_limelightTower =
      m_limelightPivot.append(new MechanismLigament2d("limelightTower", 10, 90));
  private final MechanismLigament2d m_limelight =
      m_limelightPivot.append(
          new MechanismLigament2d(
              "limelight",
              10,
              Units.radiansToDegrees(m_limelightSim.getAngleRads()),
              6,
              new Color8Bit(Color.kRed)));
    
    public double distance;
    public double offset;

    public NetworkTable table;
    private NetworkTableEntry tx;
    private NetworkTableEntry ta;
    private NetworkTableEntry tv;
    private NetworkTableEntry ty;
    private NetworkTableEntry tid;

    public Limelight () {
        m_limelightEncoder.setDistancePerPulse(1.0/15);
        // Put Mechanism 2d to SmartDashboard
        SmartDashboard.putData("Limelight Sim", m_limelightMech2d);
        m_limelightTower.setColor(new Color8Bit(Color.kBlue));
    }

        /** Update the simulation model. */
    public void simulationPeriodic() {
        // In this method, we update our simulation of what our arm is doing
        // First, we set our "inputs" (voltages)
        m_limelightSim.setInput(m_limelightMotor.get() * RobotController.getBatteryVoltage());

        // Next, we update it. The standard loop time is 20ms.
        m_limelightSim.update(0.020);

        // Finally, we set our simulated encoder's readings and simulated battery voltage
        m_limelightEncoderSim.setDistance(m_limelightSim.getAngleRads());
        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(m_limelightSim.getCurrentDrawAmps()));

        // Update the Mechanism Arm angle based on the simulated arm angle
        m_limelight.setAngle(Units.radiansToDegrees(m_limelightSim.getAngleRads()));
    }

        /** Run the control loop to reach and maintain the setpoint from the preferences. */
    public void reachSetpoint(double setpoint) {
        var pidOutput =
            m_limelightController.calculate(
                m_limelightEncoder.getDistance(), Units.degreesToRadians(setpoint));
        m_limelightMotor.setVoltage(pidOutput);
        SmartDashboard.putNumber("limelightPID", pidOutput);
    }

    public void stop() {
        m_limelightMotor.set(0.0);
    }

    public double getPos() {
        return m_limelightEncoder.getDistance();
    }

    public void teleopCommand(){
        updateData();
        if(Robot.shooter.shootingMode){
            reachSetpoint(LimelightConstants.kShooterPosition);
                    SmartDashboard.putNumber("Limelight Setpoint", LimelightConstants.kShooterPosition);

        } else {
            reachSetpoint(LimelightConstants.kIntakePosition);
            SmartDashboard.putNumber("Limelight Setpoint", LimelightConstants.kIntakePosition);
        }

        SmartDashboard.putNumber("Limelight Position", Robot.limelight.getPos()/(2*Math.PI)*360);
        limelightLog();
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

    public void setPipeline(double pipeline) {
        table.getEntry("pipeline").setNumber(pipeline);
    }

    public void limelightLog(){
        Logger.info("LMLGT", Double.toString(getPos()) + " Degrees");
        //if(limelightRotator.getFaults()!=0){Logger.warn("LMLGT: " + Short.toString(limelightRotator.getFaults()));}
    }
    
    @Override
    public void close() {
      m_limelightMotor.close();
      m_limelightEncoder.close();
      m_limelightMech2d.close();
      m_limelightPivot.close();
      m_limelightController.close();
      m_limelight.close();
    }
}
