package frc.robot.commands.Autonomous.Autonomous_Actions;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.commands.Autonomous.Subsystem_Commands.AutoIntakeElbowSet;
import frc.robot.commands.Autonomous.Subsystem_Commands.AutoIntakeWristSet;
import frc.robot.commands.Autonomous.Subsystem_Commands.AutoIntake_Trigger;
import frc.robot.commands.Autonomous.Subsystem_Commands.AutoLimelightRotate;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoAmp extends SequentialCommandGroup{

    /** Creates a new Drivetrain Snap-to-angle Command. */
    public AutoAmp() {
        addCommands(
            //Rotate elbow and bot to amp scoring position
            new ParallelCommandGroup(
                new AutoLimelightRotate(LimelightConstants.kIntakePosition),
                new AutoIntakeElbowSet(IntakeConstants.kElbowAmp)
            ),
            //Align to amp, drop arm into amp
            new ParallelCommandGroup(
                new AutoIntakeWristSet(IntakeConstants.kWristAmp)
            ),
            //Score
            new AutoIntake_Trigger(0.5,true)
        );
    }
}