package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.CANID;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Robot;
import frc.robot.ControlConfigs.PlayerConfigs;

public class Shooter implements AutoCloseable {
    public boolean prev_ActiveButton;
    public boolean shootingMode;
    public double setpoint;

    // The arm gearbox represents a gearbox containing two Vex 775pro motors.
    private final DCMotor m_flywheelGearbox = DCMotor.getNEO(1);

    // Standard classes for controlling our arm
    SimpleMotorFeedforward m_shooterFeedforward = new SimpleMotorFeedforward(0.0, ShooterConstants.kShooterFF, 0.0);
    private final PIDController m_flywheelController = new PIDController(ShooterConstants.kShooterP, 0, 0);
    public final Encoder m_flywheelEncoder =
        new Encoder(6, 7);
    private final PWMSparkMax m_flywheelMotor = new PWMSparkMax(CANID.FLYWHEEL_LEFT);

    // Simulation classes help us simulate what's going on, including gravity.
    // This arm sim represents an arm that can travel from -75 degrees (rotated down front)
    // to 255 degrees (rotated down in the back).
    private final FlywheelSim m_flywheelSim =
        new FlywheelSim(
            m_flywheelGearbox,
            1.0,
            0.0002341117);

    private final EncoderSim m_flywheelEncoderSim = new EncoderSim(m_flywheelEncoder);
    private final PWMSim m_flywheelMotorSim = new PWMSim(m_flywheelMotor);


    /** Subsystem constructor. */
    public Shooter() {
        m_flywheelEncoder.setDistancePerPulse(1.0);
    }

    /** Update the simulation model. */
    public void simulationPeriodic() {
        // In this method, we update our simulation of what our elevator is doing
        // First, we set our "inputs" (voltages)
        m_flywheelSim.setInput(m_flywheelMotorSim.getSpeed() * RobotController.getBatteryVoltage());

        // Next, we update it. The standard loop time is 20ms.
        m_flywheelSim.update(0.020);

        // Finally, we set our simulated encoder's readings and simulated battery voltage
        m_flywheelEncoderSim.setRate(m_flywheelSim.getAngularVelocityRPM());
        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(m_flywheelSim.getCurrentDrawAmps()));
    }

    /** Run the control loop to reach and maintain the setpoint from the preferences. */
    public void reachSetpoint(double RPM) {
        double closedLoopOutput = m_shooterFeedforward.calculate(RPM) + m_flywheelController.calculate(m_flywheelEncoder.getRate(), RPM);
    m_flywheelMotor.setVoltage(closedLoopOutput);
    SmartDashboard.putNumber("flywheelSetpoint", closedLoopOutput);
    }

    public void stop() {
        m_flywheelMotor.set(0.0);
    }

    public void shooterLog(){
        //Logger.info("SHOOT", Double.toString(getSpeed()) + " RPM");
        //if(flywheelL.getFaults()!=0){Logger.warn("FWLFT: " + Short.toString(flywheelL.getFaults()));}
        //if(flywheelR.getFaults()!=0){Logger.warn("FWRGT: " + Short.toString(flywheelR.getFaults()));}
    }

    public double getSpeed(){
        return m_flywheelEncoder.getRate();
    }

    public void teleopCommand(){
        if(PlayerConfigs.shooterActive != prev_ActiveButton){
            prev_ActiveButton = PlayerConfigs.shooterActive;
            if(PlayerConfigs.shooterActive){
                Robot.shooter.shootingMode = !Robot.shooter.shootingMode;
            }
        }

        if(Robot.shooter.shootingMode == true && PlayerConfigs.armScoringMechanism == true){
            setpoint = Constants.ShooterConstants.kArmedRPM;
        } else if(Robot.shooter.shootingMode == true){
            setpoint = ShooterConstants.kPrimeRPM;
        } else {
            setpoint = 0;
        }

        Robot.shooter.reachSetpoint(setpoint);
        Robot.shooter.shooterLog();
    }

    @Override
    public void close() {
        m_flywheelMotor.close();
        m_flywheelEncoder.close();
        m_flywheelController.close();
    }
}
