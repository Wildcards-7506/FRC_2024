package frc.robot.commands.Autonomous.Autonomous_Actions;

import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.Autonomous.Subsystem_Commands.AutoIntakeElbowSet;
import frc.robot.commands.Autonomous.Subsystem_Commands.AutoIntakeWristSet;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoIntakeStowToAmp extends SequentialCommandGroup{

    public AutoIntakeStowToAmp() {
        addCommands(
            new ParallelCommandGroup(
                new AutoIntakeWristSet(170,2),
                new AutoIntakeElbowSet(IntakeConstants.kElbowAmp,2)
            ),            
            new AutoIntakeWristSet(IntakeConstants.kWristAmp, 5)
        );
    }
}