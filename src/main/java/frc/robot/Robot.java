// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.IntakeConstants;
import frc.robot.ControlConfigs.PlayerConfigs;
import frc.robot.ControlConfigs.Drivers.Jayden;
import frc.robot.ControlConfigs.Drivers.Ricardo;
import frc.robot.ControlConfigs.Drivers.Ryan;
import frc.robot.ControlConfigs.Drivers.TestController;
import frc.robot.commands.ClimberTeleopCommand;
import frc.robot.commands.DrivetrainTeleopCommand;
import frc.robot.commands.LEDTeleopCommand;
import frc.robot.commands.LimelightTeleopCommand;
import frc.robot.commands.ShooterTeleopCommand;
import frc.robot.commands.Autonomous.AutoRoutines;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LEDs;
import frc.robot.util.Logger;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {  
  //Modes and people
  private AutoRoutines autoMode;
  public PlayerConfigs driver;
  public PlayerConfigs coDriver;

  public static SendableChooser<PlayerConfigs> driver_chooser = new SendableChooser<>();
  public static SendableChooser<PlayerConfigs> coDriver_chooser = new SendableChooser<>();

  public static PlayerConfigs ryan = new Ryan();
  public static PlayerConfigs jayden = new Jayden();
  public static PlayerConfigs ricardo = new Ricardo();
  public static PlayerConfigs test = new TestController();
  
  //Subsystem Declarations
  public static final Drivetrain drivetrain = new Drivetrain();
  public static final Intake intake = new Intake();
  public static final Shooter shooter = new Shooter();
  public static final Climbers climbers = new Climbers();  
  public static final Limelight limelight = new Limelight();
  public static final LEDs ledSystem = new LEDs();

  //Controllers
  public static final XboxController controller0 = new XboxController(Constants.IOConstants.DRIVER_CONTROLLER_0);
  public static final XboxController controller1 = new XboxController(Constants.IOConstants.DRIVER_CONTROLLER_1);

  //Test Timer & Field Info
  Timer timer = new Timer();
  public static Optional<Alliance> teamColor;
  public final static Field2d m_field = new Field2d();

  /*
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Driver choosers
    driver_chooser.setDefaultOption("Ricardo", ricardo);
    driver_chooser.addOption("Jayden", jayden);
    driver_chooser.addOption("Ryan", ryan);  
    driver_chooser.addOption("Test", test);       

    // Co-Driver choosers
    coDriver_chooser.setDefaultOption("Jayden", jayden);
    coDriver_chooser.addOption("Ricardo", ricardo);
    coDriver_chooser.addOption("Ryan", ryan);    
    coDriver_chooser.addOption("Test", test);  

    // Put the choosers on the dashboard
    SmartDashboard.putData(driver_chooser);
    SmartDashboard.putData(coDriver_chooser);
    SmartDashboard.putBoolean("Confirm Alliance", false);
    SmartDashboard.putData(m_field);

    Logger.info("SYSTEM","Robot Started");
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Match Time",Timer.getMatchTime());
  }

  @Override
  public void autonomousInit() {
    Logger.info("SYSTEM","Autonomous Program Started");
    CommandScheduler.getInstance().cancelAll();
    autoMode.resetAutoHeading();
    autoMode.getAutonomousCommand().schedule();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
    ledSystem.rainbow();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    Logger.info("SYSTEM","Teleop Started");
    CommandScheduler.getInstance().cancelAll();
    driver = driver_chooser.getSelected();
    coDriver = coDriver_chooser.getSelected();
    Robot.drivetrain.setDefaultCommand(new DrivetrainTeleopCommand());
    Robot.climbers.setDefaultCommand(new ClimberTeleopCommand());
    Robot.shooter.setDefaultCommand(new ShooterTeleopCommand());
    Robot.ledSystem.setDefaultCommand(new LEDTeleopCommand());
    Robot.limelight.setDefaultCommand(new LimelightTeleopCommand());
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
    driver.getDriverConfig();
    coDriver.getCoDriverConfig();

    //Intake
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

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    Logger.info("SYSTEM", "Robot Disabled");
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    teamColor = DriverStation.getAlliance();
    if(!teamColor.isEmpty()){
      if(teamColor.get() == Alliance.Red && autoMode == null && SmartDashboard.getBoolean("Confirm Alliance", false)){
        autoMode = new AutoRoutines(true);
      } else if(teamColor.get() == Alliance.Blue && autoMode == null && SmartDashboard.getBoolean("Confirm Alliance", false)){
        autoMode = new AutoRoutines(false);
      }
    }
    ledSystem.rainbow();
    
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    intake.simulationPeriodic();
  }
}
