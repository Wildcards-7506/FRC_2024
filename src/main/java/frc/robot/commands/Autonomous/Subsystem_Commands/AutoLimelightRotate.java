package frc.robot.commands.Autonomous.Subsystem_Commands;

import frc.robot.Robot;
import frc.robot.util.Logger;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoLimelightRotate extends Command{
    
    double setpoint;

    /** Creates a new Drivetrain Snap-to-angle Command. */
    public AutoLimelightRotate(double setpoint) {
        this.setpoint = setpoint;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Logger.info("LLROT", "Limelight Moving");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(!Robot.skipNonPath){
            Robot.limelight.limelightSetpoint = setpoint;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Logger.info("LLROT", "Limelight In Position");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return  Robot.skipNonPath || Math.abs(setpoint - Robot.limelight.getPos()) < 5;
    }
}