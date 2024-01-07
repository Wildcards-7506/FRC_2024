package frc.robot.commands.Autonomous.Subsystem_Commands;

import frc.robot.Robot;
import frc.robot.util.Logger;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoStrafe extends Command{
    
    double setpoint;
    double runTime;
    Timer time = new Timer();
    
    /** Creates a new Auto Pitch Correction Command. */
    public AutoStrafe(double setPoint, double runTime) {
        this.setpoint = setPoint;
        this.runTime = runTime;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Logger.info("STRAFE","Strafe Started");
        time.reset();
        time.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {        
        Robot.drivetrain.drive(0, this.setpoint, 0, true,true);
        Logger.info("STRAFE",Double.toString(time.get()) + " Seconds");
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Logger.info("STRAFE","Strafe Complete");
        Robot.drivetrain.drive(0,0,0,true,false);   
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return time.get() > this.runTime;
    }
}