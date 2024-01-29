package frc.robot.commands.Autonomous.Autonomous_Actions;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.commands.Autonomous.Subsystem_Commands.AutoIntakeElbowSet;
import frc.robot.commands.Autonomous.Subsystem_Commands.AutoIntakeWristSet;
import frc.robot.commands.Autonomous.Subsystem_Commands.AutoLimelightRotate;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoAmpSet extends SequentialCommandGroup{

    /** Creates a new Drivetrain Snap-to-angle Command. */
    public AutoAmpSet() {
        addCommands(
            //Rotate elbow and bot to amp scoring position
            new ParallelCommandGroup(
                new AutoLimelightRotate(LimelightConstants.kIntakePosition),
                new AutoIntakeElbowSet(IntakeConstants.kElbowAmp),
                new AutoIntakeWristSet(IntakeConstants.kWristAmp, 5)
            )
        );
    }
}