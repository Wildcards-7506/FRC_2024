package frc.robot.commands.Autonomous.Subsystem_Commands;

import frc.robot.Robot;
import frc.robot.util.Logger;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoIntakeElbowSet extends Command{
    
    double setpoint;

    /** Creates a new Drivetrain Snap-to-angle Command. */
    public AutoIntakeElbowSet(double setpoint) {
        this.setpoint = setpoint;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Logger.info("ELBOW", "Elbow Moving");
    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Logger.info("ELBOW", Double.toString(Robot.intake.getElbowEncoder()) + " Degrees");
        Robot.intake.setElbowPosition(setpoint);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Logger.info("ELBOW", "Elbow In Position");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return  Math.abs(setpoint - Robot.intake.getElbowEncoder()) < 5;
    }
}