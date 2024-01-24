package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CANID;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Robot;
import frc.robot.ControlConfigs.PlayerConfigs;
import frc.robot.util.Logger;

public class Climbers implements AutoCloseable {

  // The arm gearbox represents a gearbox containing two Vex 775pro motors.
  private final DCMotor m_climberGearbox = DCMotor.getNEO(2);

  // Standard classes for controlling our arm
  public final Encoder m_climberEncoder =
      new Encoder(4, 5);
  private final PWMSparkMax m_climberMotor = new PWMSparkMax(CANID.CLIMBER_LEFT);

  // Simulation classes help us simulate what's going on, including gravity.
  // This arm sim represents an arm that can travel from -75 degrees (rotated down front)
  // to 255 degrees (rotated down in the back).
  private final ElevatorSim m_climberSim =
     new ElevatorSim(
        m_climberGearbox,
        24,
        0.1,
        0.1,
        0,
        28,
        false,
        0,
        VecBuilder.fill(0.01));

  private final EncoderSim m_climberEncoderSim = new EncoderSim(m_climberEncoder);
  private final PWMSim m_climberMotorSim = new PWMSim(m_climberMotor);

  // Create a Mechanism2d visualization of the elevator
  private final MechanismRoot2d m_mech2dRoot = Robot.m_Mech2d.getRoot("Climber Root", 7, 6);
  private final MechanismLigament2d m_climberMech2d =
      m_mech2dRoot.append(
          new MechanismLigament2d("Climber", m_climberSim.getPositionMeters(), 90));

  /** Subsystem constructor. */
  public Climbers() {
    m_climberEncoder.setDistancePerPulse(ClimberConstants.kClimberEncoderDistancePerPulse);
  }

  /** Update the simulation model. */
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    m_climberSim.setInput(m_climberMotorSim.getSpeed() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_climberSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_climberEncoderSim.setDistance(m_climberSim.getPositionMeters());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_climberSim.getCurrentDrawAmps()));
  }

  /** Run the control loop to reach and maintain the setpoint from the preferences. */
  public void reachSetpoint(double voltage) {
    m_climberMotor.setVoltage(voltage);
    SmartDashboard.putNumber("climberSetpoint", voltage);
  }

  public void stop() {
    m_climberMotor.set(0.0);
  }

    /** Update telemetry, including the mechanism visualization. */
    public void updateTelemetry() {
        // Update elevator visualization with position
        m_climberMech2d.setLength(28-m_climberEncoder.getDistance());
      }

    public double getClimberEncoder() {
        return m_climberEncoder.getDistance();
    }

    public void climberLog(){
        Logger.info("CLIMB", Double.toString(getClimberEncoder()) + " Inches");
    }

    public void teleopCommand(){
        if(!Robot.shooter.shootingMode && m_climberEncoder.getDistance() >= 0 && m_climberEncoder.getDistance() <= 28){
            if (PlayerConfigs.climberDown) {
                reachSetpoint(-12);
            } else if (PlayerConfigs.climberUp) {
                reachSetpoint(12);
            } else {
                reachSetpoint(0);
            }
        }
        updateTelemetry();
        Robot.climbers.climberLog();
        SmartDashboard.putNumber("Climber Position", getClimberEncoder());
    }
        
    @Override
    public void close() {
      m_climberMotor.close();
      m_climberEncoder.close();
      m_climberMech2d.close();
    }
}