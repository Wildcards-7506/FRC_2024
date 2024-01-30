package frc.robot.commands.Autonomous.Autonomous_Actions;

import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.Autonomous.Subsystem_Commands.AutoIntakeElbowSet;
import frc.robot.commands.Autonomous.Subsystem_Commands.AutoIntakeWristSet;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoIntakeAmpToGround extends SequentialCommandGroup{

    /** Creates a new Drivetrain Snap-to-angle Command. */
    public AutoIntakeAmpToGround() {
        //Sets intake at stow, moves elbow to ground, drops intake to ground
        addCommands(
            new AutoIntakeWristSet(IntakeConstants.kWristConstraint, 5),
            new AutoIntakeElbowSet(IntakeConstants.kElbowGround, 40),
            new AutoIntakeWristSet(IntakeConstants.kWristGround,5)
        );
    }
}