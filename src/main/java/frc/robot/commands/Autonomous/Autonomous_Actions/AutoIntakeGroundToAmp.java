package frc.robot.commands.Autonomous.Autonomous_Actions;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.commands.Autonomous.Subsystem_Commands.AutoIntakeElbowSet;
import frc.robot.commands.Autonomous.Subsystem_Commands.AutoIntakeWristSet;
import frc.robot.commands.Autonomous.Subsystem_Commands.AutoLimelightRotate;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoIntakeGroundToAmp extends SequentialCommandGroup{

    public AutoIntakeGroundToAmp() {
        //sets intake to stow, lifts elbow to stow
        addCommands(
            new AutoLimelightRotate(LimelightConstants.kIntakePosition),
            new AutoIntakeElbowSet(IntakeConstants.kElbowAmp, 10),
            new AutoIntakeWristSet(IntakeConstants.kWristAmp,10)
        );
    }
}