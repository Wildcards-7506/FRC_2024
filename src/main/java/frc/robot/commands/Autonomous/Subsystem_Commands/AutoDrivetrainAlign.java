package frc.robot.commands.Autonomous.Subsystem_Commands;

import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.Logger;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoDrivetrainAlign extends Command{

    /** Creates a new Auto Pitch Correction Command. */
    public AutoDrivetrainAlign() {
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Logger.info("ALIGN","Align Started");
        if(Robot.teamColor.get() == Alliance.Red){
            Robot.limelight.setPipeline(4.0);
        } else {
            Robot.limelight.setPipeline(7.0);
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {    
        Robot.limelight.updateData();    
        Robot.drivetrain.drive(0, Robot.limelight.getTX() * DriveConstants.kAlignKP, 0, false,true);
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
        return Math.abs(Robot.limelight.getTX()) < 1;
    }
}