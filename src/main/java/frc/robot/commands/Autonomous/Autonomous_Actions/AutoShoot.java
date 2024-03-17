package frc.robot.commands.Autonomous.Autonomous_Actions;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Autonomous.Subsystem_Commands.AutoIntakeElbowSet;
import frc.robot.commands.Autonomous.Subsystem_Commands.AutoIntakeWristSet;
import frc.robot.commands.Autonomous.Subsystem_Commands.AutoIntake_Trigger;
import frc.robot.commands.Autonomous.Subsystem_Commands.AutoShooterSpinUp;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoShoot extends SequentialCommandGroup{

    public AutoShoot() {
        addCommands(
            new AutoShooterSpinUp(ShooterConstants.kLArmedRPM, ShooterConstants.kRArmedRPM),
            new AutoIntake_Trigger(0.5,true),
            new ParallelCommandGroup(
                new AutoIntakeElbowSet(IntakeConstants.kElbowStowed, 10),
                new AutoIntakeWristSet(IntakeConstants.kWristStowed,10)
            )
        );
    }
}