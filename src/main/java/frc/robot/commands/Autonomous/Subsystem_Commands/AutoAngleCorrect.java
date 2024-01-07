package frc.robot.commands.Autonomous.Subsystem_Commands;

import frc.robot.Robot;
import frc.robot.util.Logger;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoAngleCorrect extends Command{
    
    boolean tilted = false;
    double angle;
    double driveSpeed;
    Timer time;

    /** Creates a new Auto ANGLE Correction Command. */
    public AutoAngleCorrect() {}

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Logger.info("ANGLE", "Balance Started");
        driveSpeed = -0.6;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        angle = Robot.drivetrain.getRoll();
        Logger.info("ANGLE", Double.toString(angle) + " Degrees");
        if(angle < -3 || angle > 3){
            tilted = true;
            driveSpeed = 0.23 * angle/15;
            time.reset();
        } else if(tilted) {
            time.start();
        }
        Robot.drivetrain.drive(driveSpeed, 0.0, 0, false,false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Logger.info("ANGLE", "Balance Complete");
        Robot.drivetrain.setX();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return time.get() > 1;
    }
}