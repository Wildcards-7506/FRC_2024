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
import frc.robot.Constants.CANID;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Robot;
import frc.robot.ControlConfigs.PlayerConfigs;

public class Intake implements AutoCloseable {
    public double elbowSetPoint = IntakeConstants.kElbowStowed;
    public double wristSetPoint;
    public double intakeState;
    public boolean pieceAcquired;

  // The arm gearbox represents a gearbox containing two Vex 775pro motors.
  private final DCMotor m_elbowGearbox = DCMotor.getNEO(2);
  private final DCMotor m_wristGearbox = DCMotor.getNEO(1);

  // Standard classes for controlling our arm
  private final PIDController m_elbowController = new PIDController(IntakeConstants.kElbowKP, 0, 0.5);
  public final Encoder m_elbowEncoder =
      new Encoder(0, 1);
  private final PWMSparkMax m_elbowMotor = new PWMSparkMax(CANID.ELBOW_LEFT);
  private final PIDController m_wristController = new PIDController(IntakeConstants.kWristKP, 0, 0.5);
  public final Encoder m_wristEncoder =
      new Encoder(2, 3);
  private final PWMSparkMax m_wristMotor = new PWMSparkMax(CANID.WRIST);

  // Simulation classes help us simulate what's going on, including gravity.
  // This arm sim represents an arm that can travel from -75 degrees (rotated down front)
  // to 255 degrees (rotated down in the back).
  private final SingleJointedArmSim m_elbowSim =
      new SingleJointedArmSim(
          m_elbowGearbox,
          1.0/125,
          SingleJointedArmSim.estimateMOI(0.1, 0.1),
          0.8,
          -Math.PI/4,
          155.0/360*2*Math.PI,
          false,
          0,
          VecBuilder.fill(IntakeConstants.kElbowEncoderDistancePerPulse*0.01) // Add noise with a std-dev of 1 tick
          );
  private final EncoderSim m_elbowEncoderSim = new EncoderSim(m_elbowEncoder);

  private final SingleJointedArmSim m_wristSim =
      new SingleJointedArmSim(
          m_wristGearbox,
          1.0/60,
          SingleJointedArmSim.estimateMOI(0.1, 0.1),
          0.35,
          -90.0/360*Math.PI*2,
          270.0/360*2*Math.PI,
          false,
          0,
          VecBuilder.fill(IntakeConstants.kWristEncoderDistancePerPulse*0.01) // Add noise with a std-dev of 1 tick
          );
  private final EncoderSim m_wristEncoderSim = new EncoderSim(m_wristEncoder);

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final Mechanism2d m_elbowMech2d = new Mechanism2d(60, 60);
  private final MechanismRoot2d m_elbowPivot = m_elbowMech2d.getRoot("ElbowPivot", 30, 30);
  private final MechanismLigament2d m_elbowTower =
      m_elbowPivot.append(new MechanismLigament2d("ElbowTower", 30, -90));
  private final MechanismLigament2d m_elbow =
      m_elbowPivot.append(
          new MechanismLigament2d(
              "Elbow",
              30,
              Units.radiansToDegrees(m_elbowSim.getAngleRads()),
              6,
              new Color8Bit(Color.kAqua)));

    private final Mechanism2d m_wristMech2d = new Mechanism2d(60, 60);
  private final MechanismRoot2d m_wristPivot = m_wristMech2d.getRoot("WristPivot", 30, 30);
private final MechanismLigament2d m_wristTower =
      m_wristPivot.append(new MechanismLigament2d("WristTower", 30, -90));
  private final MechanismLigament2d m_wrist =
      m_wristPivot.append(
          new MechanismLigament2d(
              "Wrist",
              20,
              Units.radiansToDegrees(m_wristSim.getAngleRads()),
              6,
              new Color8Bit(Color.kYellow)));

  /** Subsystem constructor. */
  public Intake() {
    m_elbowEncoder.setDistancePerPulse(IntakeConstants.kElbowEncoderDistancePerPulse);

    // Put Mechanism 2d to SmartDashboard
    SmartDashboard.putData("Elbow Sim", m_elbowMech2d);
    m_elbowTower.setColor(new Color8Bit(Color.kBlue));

    m_wristEncoder.setDistancePerPulse(IntakeConstants.kWristEncoderDistancePerPulse);

    // Put Mechanism 2d to SmartDashboard
    SmartDashboard.putData("Wrist Sim", m_wristMech2d);
m_wristTower.setColor(new Color8Bit(Color.kBlue));
  }

  /** Update the simulation model. */
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    m_elbowSim.setInput(m_elbowMotor.get() * RobotController.getBatteryVoltage());
    m_wristSim.setInput(m_wristMotor.get() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_elbowSim.update(0.020);
    m_wristSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_elbowEncoderSim.setDistance(m_elbowSim.getAngleRads());
    m_wristEncoderSim.setDistance(m_wristSim.getAngleRads());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_elbowSim.getCurrentDrawAmps()));
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_wristSim.getCurrentDrawAmps()));

    // Update the Mechanism Arm angle based on the simulated arm angle
    m_elbow.setAngle(Units.radiansToDegrees(m_elbowSim.getAngleRads()));
    m_wrist.setAngle(Units.radiansToDegrees(m_wristSim.getAngleRads()));
  }

  /** Run the control loop to reach and maintain the setpoint from the preferences. */
  public void reachSetpoint(double e_setpoint, double w_setpoint) {
    var e_pidOutput =
        m_elbowController.calculate(
            m_elbowEncoder.getDistance(), Units.degreesToRadians(e_setpoint));
    m_elbowMotor.setVoltage(e_pidOutput);
    SmartDashboard.putNumber("ElbowPID", e_pidOutput);

    var w_pidOutput =
        m_wristController.calculate(
            m_wristEncoder.getDistance(), Units.degreesToRadians(w_setpoint));
    m_wristMotor.setVoltage(w_pidOutput);
    SmartDashboard.putNumber("WristPID", w_pidOutput);
  }

  public void stop() {
    m_elbowMotor.set(0.0);
    m_wristMotor.set(0.0);
  }

  public void teleopCommand(){
    if(PlayerConfigs.intake){
            Robot.intake.intakeState = 1;
            Robot.intake.pieceAcquired = false;
        } else if(PlayerConfigs.amp){
            Robot.intake.intakeState = 2;
        } else if(PlayerConfigs.trap){
            Robot.intake.intakeState = 3;
        } else if(PlayerConfigs.stow){
            Robot.intake.intakeState = 0;
        }


        if (Robot.intake.m_wristEncoder.getDistance() > 170.0/360*2*Math.PI) {
            if (Robot.intake.intakeState == 1) {
                Robot.intake.elbowSetPoint = IntakeConstants.kElbowGround;
            } else if (Robot.intake.intakeState == 2) {
                Robot.intake.elbowSetPoint = IntakeConstants.kElbowAmp;
            } else if (Robot.intake.intakeState == 3) {
                Robot.intake.elbowSetPoint = IntakeConstants.kElbowTrap;
            } else {
                Robot.intake.elbowSetPoint = IntakeConstants.kElbowStowed;
            }
        }

        if (Robot.intake.intakeState == 1) {
            if (Math.abs(IntakeConstants.kElbowGround/360*2*Math.PI - Robot.intake.m_elbowEncoder.getDistance()) < 0.1) {
                Robot.intake.wristSetPoint = IntakeConstants.kWristGround;
            } else {
                Robot.intake.wristSetPoint = IntakeConstants.kWristStowed;
            }
        } else if (Robot.intake.intakeState == 2){
            if (Math.abs(IntakeConstants.kElbowAmp/360*2*Math.PI  - Robot.intake.m_elbowEncoder.getDistance()) < 0.1) {
                Robot.intake.wristSetPoint = IntakeConstants.kWristAmp;
            } else {
                Robot.intake.wristSetPoint = IntakeConstants.kWristStowed;
            }
        } else if (Robot.intake.intakeState == 3) {
            if (Math.abs(IntakeConstants.kElbowTrap/360*2*Math.PI  - Robot.intake.m_elbowEncoder.getDistance()) < 0.1 && PlayerConfigs.armScoringMechanism) {
                Robot.intake.wristSetPoint = IntakeConstants.kWristTrap;
            } else {
                Robot.intake.wristSetPoint = IntakeConstants.kWristStowed;
            }
        } else if (PlayerConfigs.armScoringMechanism) {
            if (Math.abs(IntakeConstants.kElbowStowed/360*2*Math.PI  - Robot.intake.m_elbowEncoder.getDistance()) < 0.1) {
                Robot.intake.wristSetPoint = IntakeConstants.kWristShooting;
            } else {
                Robot.intake.wristSetPoint = IntakeConstants.kWristStowed;
            }
        } else {
            Robot.intake.wristSetPoint = IntakeConstants.kWristStowed;
        }

        Robot.intake.reachSetpoint(Robot.intake.elbowSetPoint, Robot.intake.wristSetPoint);

        // if ((Robot.intake.intakeState == 1 &! Robot.intake.pieceAcquired) || ((Robot.intake.intakeState == 0 || Robot.intake.intakeState == 3) && PlayerConfigs.fire)) {
        //     Robot.intake.setIntakeVoltage(12);
        //     Robot.intake.running = Robot.intake.getIntakeSpeed() > 200 ? true : false;
        //     Robot.intake.pieceAcquired = (Robot.intake.running && Robot.intake.getIntakeCurrent() > 20) ? true : false;
        // } else {
        //     Robot.intake.setIntakeVoltage(0);
        //     Robot.intake.running = false;
        // }

        SmartDashboard.putNumber("Wrist Setpoint: ", Robot.intake.wristSetPoint);
        SmartDashboard.putNumber("Elbow Setpoint: ", Robot.intake.elbowSetPoint);
        SmartDashboard.putNumber("Wrist Position: ", Robot.intake.m_wristEncoder.getDistance()/(2*Math.PI)*360);
        SmartDashboard.putNumber("Elbow Position: ", Robot.intake.m_elbowEncoder.getDistance()/(2*Math.PI)*360);
        SmartDashboard.putNumber("Intake State: ",Robot.intake.intakeState);
        SmartDashboard.putBoolean("Piece Acquired: ", Robot.intake.pieceAcquired);
        SmartDashboard.putBoolean("error", Math.abs(IntakeConstants.kElbowTrap/360*2*Math.PI  - Robot.intake.m_elbowEncoder.getDistance()) < 0.1);
        SmartDashboard.putBoolean("Score", PlayerConfigs.armScoringMechanism);
  }

  @Override
  public void close() {
    m_elbowMotor.close();
    m_elbowEncoder.close();
    m_elbowMech2d.close();
    m_elbowPivot.close();
    m_elbowController.close();
    m_elbow.close();

    m_wristMotor.close();
    m_wristEncoder.close();
    m_wristMech2d.close();
    m_wristPivot.close();
    m_wristController.close();
    m_wrist.close();
  }
}