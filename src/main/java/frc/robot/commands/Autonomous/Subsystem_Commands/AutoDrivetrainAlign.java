package frc.robot.commands.Autonomous.Subsystem_Commands;

import frc.robot.Robot;
import frc.robot.util.Logger;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoDrivetrainAlign extends Command{
    boolean amp;
    double pipeline;

    /** Creates a new Auto Pitch Correction Command. */
    public AutoDrivetrainAlign(boolean amp) {
        this.amp = amp;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Logger.info("ALIGN","Align Started");
        if(Robot.teamColor.get() == Alliance.Red){
            pipeline = amp ? 5.0 : 4.0;
        } else {
            pipeline = amp ? 6.0 : 7.0;
        }
        Robot.limelight.setPipeline(pipeline);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {    
        Robot.limelight.updateData();    
        Robot.drivetrain.align(Robot.limelight.getTX(), Robot.limelight.getID());
        Logger.info("ALIGN",Double.toString(Robot.limelight.getTX()) + " Degrees");
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Logger.info("ALIGN","Alignment Complete");
        Robot.drivetrain.drive(0,0,0,false,false);   
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Robot.skipNonPath || Math.abs(Robot.limelight.getTX()) < 1;
    }
}