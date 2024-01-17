// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.commands.Autonomous.Autonomous_Actions.AutoAmp;
import frc.robot.commands.Autonomous.Autonomous_Actions.AutoIntakeDown;
import frc.robot.commands.Autonomous.Autonomous_Actions.AutoIntakeUp;
import frc.robot.commands.Autonomous.Autonomous_Actions.AutoShoot;
import frc.robot.commands.Autonomous.Subsystem_Commands.AutoDrivetrainSnap;
import frc.robot.commands.Autonomous.Subsystem_Commands.AutoIntake_Trigger;
import frc.robot.commands.Autonomous.Subsystem_Commands.AutoX;
import frc.robot.subsystems.Drivetrain;

@SuppressWarnings("unused")
public final class AutoRoutines {

  private HashMap<String, Command> eventMap;
  private final AutoBuilder autoBuilder = new AutoBuilder();

  // Autonomous selector on dashboard
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private double kAutoStartDelaySeconds;
  
  public AutoRoutines(boolean flip) {
    eventMap = new HashMap<>();
    setMarkers();

    AutoBuilder.configureHolonomic(
      Robot.drivetrain::getPose,
      Robot.drivetrain::resetOdometry,
      Robot.drivetrain::getRobotRelativeSpeeds,
      Robot.drivetrain::driveRobotRelative,
      new HolonomicPathFollowerConfig(
        new PIDConstants(AutoConstants.kPXController,0,0),
        new PIDConstants(AutoConstants.kPThetaController,0,0),
        4.8, // max speed in m/s
        Units.inchesToMeters(Math.sqrt(Math.pow(20.0, 2)+Math.pow(20.0,2))/2), // Radius in meters of 28.5 x 18.5 inch robot using a^2 +b^2 = c^2
        new ReplanningConfig()
      ),
      ()->flip, Robot.drivetrain
    );

    // Autonomous selector options
    kAutoStartDelaySeconds = SmartDashboard.getNumber("Auto Delay",0.0);
    
    autoChooser.setDefaultOption("Nothing", Commands.none());
    autoChooser.addOption("4 Top", AutoBuilder.buildAuto("T4PA"));
    autoChooser.addOption("4 Center", AutoBuilder.buildAuto("C4PA"));
    autoChooser.addOption("4 Bottom", AutoBuilder.buildAuto("B4PA"));
    autoChooser.addOption("3 Top", AutoBuilder.buildAuto("T3PA"));
    autoChooser.addOption("3 Bottom", AutoBuilder.buildAuto("B3PA"));
    autoChooser.addOption("2 Top", AutoBuilder.buildAuto("T2PA"));
    autoChooser.addOption("2 Center", AutoBuilder.buildAuto("C2PA"));
    autoChooser.addOption("2 Bottom", AutoBuilder.buildAuto("B2PA"));
    autoChooser.addOption("Amp Mid", AutoBuilder.buildAuto("AmpMid"));
    autoChooser.addOption("Troll", AutoBuilder.buildAuto("1+Troll"));

    SmartDashboard.putData("Auto Chooser",autoChooser);
  }

  private void setMarkers() {
    eventMap = buildEventMap();
  }

  private HashMap<String, Command> buildEventMap() {
    return new HashMap<>(
        Map.ofEntries(
          Map.entry("Checkpoint 1", Commands.print("Checkpoint 1")),
          Map.entry("Checkpoint 2", Commands.print("Checkpoint 2")),
          Map.entry("Checkpoint 3", Commands.print("Checkpoint 3")),
          Map.entry("AutoShootTop", new AutoShoot(0)),
          Map.entry("AutoShootCenter", new AutoShoot(1)),
          Map.entry("AutoShootBottom", new AutoShoot(2)),
          Map.entry("AutoIntakeDown", new AutoIntakeDown()),
          Map.entry("AutoIntakeUp", new AutoIntakeUp()),
          Map.entry("AutoIntake", new AutoIntake_Trigger(5, false)),
          Map.entry("AutoAmp", new AutoAmp())
        )
    );
  }

      /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.sequence(
      Commands.waitSeconds(kAutoStartDelaySeconds),
      autoChooser.getSelected());
  }

  public void resetAutoHeading() {
    Robot.drivetrain.zeroHeading();
  }
  }