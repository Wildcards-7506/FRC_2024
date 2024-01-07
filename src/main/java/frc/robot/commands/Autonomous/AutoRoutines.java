// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import java.util.HashMap;
import java.util.Map;

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
import frc.robot.commands.Autonomous.Autonomous_Actions.AutoBalance;
import frc.robot.commands.Autonomous.Subsystem_Commands.AutoSnap;
import frc.robot.commands.Autonomous.Subsystem_Commands.AutoX;
import frc.robot.subsystems.DriveSubsystem;

@SuppressWarnings("unused")
public final class AutoRoutines {

  private HashMap<String, Command> eventMap;
  private final AutoBuilder autoBuilder = new AutoBuilder();

  // Autonomous selector on dashboard
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private double kAutoStartDelaySeconds;
  
  public AutoRoutines() {
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
        Units.inchesToMeters(Math.sqrt(Math.pow(28.5, 2)+Math.pow(18.5,2))/2), // Radius in meters of 28.5 x 18.5 inch robot using a^2 +b^2 = c^2
        new ReplanningConfig()
      ),
      Robot.drivetrain
    );

    // Autonomous selector options
    kAutoStartDelaySeconds = SmartDashboard.getNumber("Auto Delay",0.0);
    
    autoChooser.setDefaultOption("Nothing", Commands.none());
    autoChooser.addOption("Helix", helix());
    autoChooser.addOption("Box", box());
    autoChooser.addOption("Spikes", spikes());

    SmartDashboard.putData("Auto Chooser",autoChooser);
  }

  private void setMarkers() {
    eventMap = buildEventMap();
  }

    public Command none() {
      return Commands.none();
    }

    public Command helix() {
      PathPlannerPath path1 = PathPlannerPath.fromPathFile("Helix");
      return AutoBuilder.followPathWithEvents(path1);
    }
    
    public Command box() {
      PathPlannerPath path2 = PathPlannerPath.fromPathFile("Box");
      return AutoBuilder.followPathWithEvents(path2);    
    }

    public Command spikes() {
      PathPlannerPath path3 = PathPlannerPath.fromPathFile("Spikes");
      return AutoBuilder.followPathWithEvents(path3);     
    }
  
    private HashMap<String, Command> buildEventMap() {
      return new HashMap<>(
          Map.ofEntries(
              //Map.entry("DoAction", subsystem.DoActionAutoCommand().alongWith(Commands.print("Doing Action"))),
              Map.entry("Checkpoint 1", Commands.print("Checkpoint 1")),
              Map.entry("Checkpoint 2", Commands.print("Checkpoint 2")),
              Map.entry("Checkpoint 3", Commands.print("Checkpoint 3")),
              Map.entry("AutoBalance", new AutoBalance(0)),
              Map.entry("AutoSnap0", new AutoSnap(0)),
              Map.entry("AutoSnap90", new AutoSnap(90)),
              Map.entry("AutoSnap180", new AutoSnap(180)),
              Map.entry("AutoSnap270", new AutoSnap(270)),
              Map.entry("AutoX", new AutoX())
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