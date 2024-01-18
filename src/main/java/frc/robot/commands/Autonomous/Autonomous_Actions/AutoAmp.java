// package frc.robot.commands.Autonomous.Autonomous_Actions;

// import frc.robot.Constants.IntakeConstants;
// import frc.robot.Constants.LimelightConstants;
// import frc.robot.commands.Autonomous.Subsystem_Commands.AutoDrivetrainAlign;
// import frc.robot.commands.Autonomous.Subsystem_Commands.AutoDrivetrainSnap;
// import frc.robot.commands.Autonomous.Subsystem_Commands.AutoIntakeElbowSet;
// import frc.robot.commands.Autonomous.Subsystem_Commands.AutoIntakeWristSet;
// import frc.robot.commands.Autonomous.Subsystem_Commands.AutoIntake_Trigger;
// import frc.robot.commands.Autonomous.Subsystem_Commands.AutoLimelightRotate;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// public class AutoAmp extends SequentialCommandGroup{

//     /** Creates a new Drivetrain Snap-to-angle Command. */
//     public AutoAmp() {
//         addCommands(
//             new ParallelCommandGroup(
//                 new AutoLimelightRotate(LimelightConstants.kIntakePosition),
//                 new AutoIntakeElbowSet(IntakeConstants.kElbowAmp),
//                 new AutoDrivetrainSnap(4)
//             ),
//             new ParallelCommandGroup(
//                 new AutoDrivetrainAlign(true),
//                 new AutoIntakeWristSet(IntakeConstants.kWristAmp)
//             ),
//             new AutoIntake_Trigger(0.5,true)
//         );
//     }
// }