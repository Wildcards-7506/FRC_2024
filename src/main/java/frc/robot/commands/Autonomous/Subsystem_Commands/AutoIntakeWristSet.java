package frc.robot.commands.Autonomous.Subsystem_Commands;

import frc.robot.Robot;
import frc.robot.util.Logger;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoIntakeWristSet extends Command{
    
    double setpoint;

    /** Creates a new Drivetrain Snap-to-angle Command. */
    public AutoIntakeWristSet(double setpoint) {
        this.setpoint = setpoint;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Logger.info("WRIST", "Wrist Moving");
    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Logger.info("WRIST", Double.toString(Robot.intake.getWristEncoder()) + " Degrees");
        Robot.intake.setWristPosition(setpoint);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Logger.info("WRIST", "Wrist In Position");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return  Robot.skipNonPath || Math.abs(setpoint - Robot.intake.getWristEncoder()) < 5;
    }
}