package frc.robot.commands.Autonomous.Subsystem_Commands;

import frc.robot.Robot;
import frc.robot.Constants.LEDConstants;
import frc.robot.util.Logger;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoIntakeElbowSet extends Command{
    
    double setpoint;
    double range;

    /** Creates a new Drivetrain Snap-to-angle Command. */
    public AutoIntakeElbowSet(double setpoint, double range) {
        this.setpoint = setpoint;
        this.range = range;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Logger.info("ELBOW", "Elbow Moving");
    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(!Robot.skipNonPath){  
            Logger.info("ELBOW", Double.toString(Robot.intake.getElbowEncoder()) + " Degrees");
            Robot.intake.setElbowPosition(setpoint);
            Robot.lightStrip.section(LEDConstants.bufferSize/5, 2*LEDConstants.bufferSize/5-1, Robot.lightStrip.shooterLo, LEDConstants.SATURATED, LEDConstants.FULL);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Logger.info("ELBOW", "Elbow In Position");
        Robot.lightStrip.section(LEDConstants.bufferSize/5, 2*LEDConstants.bufferSize/5-1, LEDConstants.GREEN, LEDConstants.SATURATED, LEDConstants.FULL);

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return  Robot.skipNonPath || Math.abs(setpoint - Robot.intake.getElbowEncoder()) < range;
    }
}