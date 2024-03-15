package frc.robot.commands.Autonomous.Autonomous_Actions;

import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.Autonomous.Subsystem_Commands.AutoIntakeElbowSet;
import frc.robot.commands.Autonomous.Subsystem_Commands.AutoIntakeWristSet;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoIntakeAmpToGround extends SequentialCommandGroup{

    public AutoIntakeAmpToGround() {
        //Sets intake at stow, moves elbow to ground, drops intake to ground
        addCommands(
            new AutoIntakeWristSet(IntakeConstants.kWristGround,60),
            new AutoIntakeElbowSet(IntakeConstants.kElbowGround, 10)
        );
    }
}