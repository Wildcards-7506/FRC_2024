package frc.robot.commands.Autonomous.Autonomous_Actions;

import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.Autonomous.Subsystem_Commands.AutoIntakeElbowSet;
import frc.robot.commands.Autonomous.Subsystem_Commands.AutoIntakeWristSet;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoIntakeStowToGround extends SequentialCommandGroup{

    public AutoIntakeStowToGround() {
        //Sets intake at stow, moves elbow to ground, drops intake to ground
        addCommands(
            new AutoIntakeWristSet(IntakeConstants.kWristStowed,5),
            new AutoIntakeElbowSet(IntakeConstants.kElbowDownConstraint, 20),
            new AutoIntakeWristSet(IntakeConstants.kWristConstraint,5),
            new AutoIntakeWristSet(IntakeConstants.kWristGround,60),
            new AutoIntakeElbowSet(IntakeConstants.kElbowGround, 10)
        );
    }
}  