package frc.robot.commands.Autonomous.Subsystem_Commands;

import frc.robot.Robot;
import frc.robot.util.Logger;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoSnap extends Command{
    
    int angle;
    double rotation;

    /** Creates a new Drivetrain Snap-to-angle Command. */
    public AutoSnap(int angle) {
        this.angle = angle;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Logger.info("DSNAP", "Snap Started");
    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double rotation = -0.1 * Math.abs(angle-Robot.drivetrain.getHeading())/(angle-Robot.drivetrain.getHeading());
        Logger.info("DSNAP", Double.toString(angle-Robot.drivetrain.getHeading()) + " Degrees");
        Robot.drivetrain.drive(0.0, 0.0, rotation, false,false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Logger.info("DSNAP", "Snap Complete");
        Robot.drivetrain.drive(0,0,0,true,false);
        Robot.drivetrain.resetEncoders();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return  (Math.abs(angle-Robot.drivetrain.getHeading()) < 1);
    }
}