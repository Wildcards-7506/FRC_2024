package frc.robot.commands.Autonomous.Subsystem_Commands;

import frc.robot.Constants.LEDConstants;
import frc.robot.Robot;
import frc.robot.util.Logger;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoIntakeWristSet extends Command{
    
    double setpoint;
    double range;

    /** Creates a new Drivetrain Snap-to-angle Command. */
    public AutoIntakeWristSet(double setpoint, double range) {
        this.setpoint = setpoint;
        this.range = range;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Logger.info("WRIST", "Wrist Moving");
    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(!Robot.skipNonPath){  
            Logger.info("WRIST", Double.toString(Robot.intake.getWristEncoder()) + " Degrees");
            Robot.intake.setWristPosition(setpoint);
            Robot.lightStrip.section(2*LEDConstants.bufferSize/5, 3*LEDConstants.bufferSize/5-1, Robot.lightStrip.shooterLo, LEDConstants.SATURATED, LEDConstants.FULL);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Logger.info("WRIST", "Wrist In Position");
        Robot.lightStrip.section(2*LEDConstants.bufferSize/5, 3*LEDConstants.bufferSize/5-1, LEDConstants.GREEN, LEDConstants.SATURATED, LEDConstants.FULL);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return  Robot.skipNonPath || Math.abs(setpoint - Robot.intake.getWristEncoder()) < range;
    }
}