package frc.robot.commands.Autonomous.Subsystem_Commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
import frc.robot.util.Logger;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoDrivetrainSnap extends Command{
    
    double location;
    double angle;
    double output;

    /** Creates a new Drivetrain Snap-to-angle Command. */
    public AutoDrivetrainSnap(int location) {
        this.location = location;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Logger.info("DSNAP", "Snap Started");
        if(Robot.teamColor.get() == Alliance.Red){
            angle = location == 0 ? -60 : location == 1 ? 0 : location == 2 ? 60 : 90;
        } else{
            angle = location == 0 ? -120 : location == 1 ? 180 : location == 2 ? 120 : 90;
        }
        if(!Robot.skipNonPath){Logger.info("DSNAP", "Setpoint: " + Double.toString(angle) + " Degrees");}
    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(!Robot.skipNonPath){  
            output = Robot.drivetrain.getHeading() - angle > 360 - (Robot.drivetrain.getHeading() - angle) ? 
                    (Robot.drivetrain.getHeading() - angle) * DriveConstants.kSnapSpeed : 
                    (Robot.drivetrain.getHeading() - angle - 360) * DriveConstants.kSnapSpeed;
            Logger.info("DSNAP", Double.toString(output/DriveConstants.kSnapSpeed) + " Degrees");
            Robot.drivetrain.drive(0.0, 0.0, output, false,false);
        }
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
        return  Robot.skipNonPath || Math.abs(output) < 1;
    }
}