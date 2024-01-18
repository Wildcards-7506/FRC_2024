// package frc.robot.commands.Autonomous.Autonomous_Actions;

// import frc.robot.Constants.IntakeConstants;
// import frc.robot.Constants.LimelightConstants;
// import frc.robot.Constants.ShooterConstants;
// import frc.robot.commands.Autonomous.Subsystem_Commands.AutoDrivetrainAlign;
// import frc.robot.commands.Autonomous.Subsystem_Commands.AutoDrivetrainSnap;
// import frc.robot.commands.Autonomous.Subsystem_Commands.AutoIntakeWristSet;
// import frc.robot.commands.Autonomous.Subsystem_Commands.AutoIntake_Trigger;
// import frc.robot.commands.Autonomous.Subsystem_Commands.AutoLimelightRotate;
// import frc.robot.commands.Autonomous.Subsystem_Commands.AutoShooterSpinUp;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// public class AutoShoot extends SequentialCommandGroup{

//     /** Creates a new Drivetrain Snap-to-angle Command. */
//     public AutoShoot(int location) {
//         addCommands(
//             new ParallelCommandGroup(
//                 new AutoLimelightRotate(LimelightConstants.kShooterPosition),
//                 new AutoIntakeWristSet(IntakeConstants.kWristShooting),
//                 new AutoShooterSpinUp(ShooterConstants.kArmedRPM),
//                 new AutoDrivetrainSnap(location)
//             ),
//             new AutoDrivetrainAlign(false),
//             new AutoIntake_Trigger(0.5,true)
//         );
//     }
// }