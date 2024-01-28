package frc.robot.commands.Autonomous.Subsystem_Commands;

import frc.robot.Robot;
import frc.robot.util.Logger;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoShooterSpinUp extends Command{
    
    double lSpeed;
    double rSpeed;

    /** Creates a new Drivetrain Snap-to-angle Command. */
    public AutoShooterSpinUp(double lSpeed) {
        this.lSpeed = lSpeed;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Logger.info("SHOOT", "Shooter Spin Up Started");
    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(!Robot.skipNonPath){  
            Logger.info("SHOOT", Double.toString(Robot.shooter.getSpeed()));
            Robot.shooter.reachSetpoint(lSpeed);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Logger.info("SHOOT", "Shooter Up To Speed");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return  Robot.skipNonPath || lSpeed - Robot.shooter.getSpeed() < 100;
    }
}