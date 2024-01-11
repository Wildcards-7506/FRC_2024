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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.ControlConfigs.PlayerConfigs;
import frc.robot.ControlConfigs.Drivers.Jayden;
import frc.robot.ControlConfigs.Drivers.Ricardo;
import frc.robot.ControlConfigs.Drivers.Ryan;
import frc.robot.commands.DrivetrainTeleopCommand;
import frc.robot.commands.LEDTeleopCommand;
import frc.robot.commands.LimelightTeleopCommand;
import frc.robot.commands.Autonomous.AutoRoutines;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightRotator;
import frc.robot.subsystems.LEDs;
import frc.robot.util.Logger;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {  
  private AutoRoutines autoMode = new AutoRoutines();
  public PlayerConfigs driver;
  public PlayerConfigs coDriver;
  //Subsystem Declarations

  public static final Drivetrain drivetrain = new Drivetrain();
  public static final Intake intake = new Intake();
  public static final Shooter shooter = new Shooter();
  public static final Climbers climbers = new Climbers();
  public static final LimelightRotator llrotator = new LimelightRotator();
  
  public static final Limelight limelight = new Limelight();

  public static final LEDs ledSystem = new LEDs(0,30);

  //Controllers
  public static final XboxController controller0 = new XboxController(Constants.IOConstants.DRIVER_CONTROLLER_0);
  public static final XboxController controller1 = new XboxController(Constants.IOConstants.DRIVER_CONTROLLER_1);

  //Test Timer & Flag
  Timer timer = new Timer();

  public static Optional<Alliance> teamColor;

  public static SendableChooser<PlayerConfigs> driver_chooser = new SendableChooser<>();
  public static SendableChooser<PlayerConfigs> coDriver_chooser = new SendableChooser<>();

  public static PlayerConfigs ryan = new Ryan();
  public static PlayerConfigs anthony = new Jayden();
  public static PlayerConfigs ricardo = new Ricardo();

  /*
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Driver choosers
    driver_chooser.setDefaultOption("Ryan", ryan);
    driver_chooser.addOption("Anthony", anthony);
    driver_chooser.addOption("Ricardo", ricardo);        

    // Co-Driver choosers
    coDriver_chooser.setDefaultOption("Anthony", anthony);
    coDriver_chooser.addOption("Ricardo", ricardo);
    coDriver_chooser.addOption("Ryan", ryan);        

    // Put the choosers on the dashboard
    SmartDashboard.putData(driver_chooser);
    SmartDashboard.putData(coDriver_chooser);

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
    teamColor = DriverStation.getAlliance();
    CommandScheduler.getInstance().cancelAll();
    //Need LED Indicator Here
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
    teamColor = DriverStation.getAlliance();
    driver = driver_chooser.getSelected();
    coDriver = coDriver_chooser.getSelected();
    Robot.drivetrain.setDefaultCommand(new DrivetrainTeleopCommand());
    Robot.ledSystem.setDefaultCommand(new LEDTeleopCommand());
    Robot.limelight.setDefaultCommand(new LimelightTeleopCommand());
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
    driver.getDriverConfig();
    coDriver.getCoDriverConfig();
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
  public void simulationPeriodic() {}
}
