package frc.robot.commands.Autonomous.Subsystem_Commands;

import frc.robot.Robot;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.util.Logger;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoIntake_Trigger extends Command{
    
    double runTime;
    double intakeTime = 0;
    boolean shooting;
    Timer time = new Timer();

    /** Creates a new Drivetrain Snap-to-angle Command. */
    public AutoIntake_Trigger(double runTime, boolean shooting) {
        this.runTime = runTime;
        this.shooting = shooting;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Logger.info("INTKE", "Intaking");
        if(shooting){
            Robot.intake.setIntakeCurrentLimit(IntakeConstants.kIntakeShootingLimit);
        } else{
            Robot.intake.setIntakeCurrentLimit(IntakeConstants.kIntakeCurrentLimit);
        }
        time.reset();
        time.start();
    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(!Robot.skipNonPath){  
            Robot.lightStrip.section(3*LEDConstants.bufferSize/5,4*LEDConstants.bufferSize/5-1,Robot.lightStrip.shooterLo,LEDConstants.SATURATED,LEDConstants.FULL);
            Logger.info("INTKE", Double.toString(Robot.intake.getIntakeCurrent()) + " Amps");
            Robot.intake.setIntakeVoltage(12);
            if(time.get() > 1 && Robot.intake.getIntakeCurrent() > 24){
                intakeTime = time.get();
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if(!Robot.skipNonPath){  
            if(time.get() > runTime){
                Robot.lightStrip.section(3*LEDConstants.bufferSize/5,4*LEDConstants.bufferSize/5-1,LEDConstants.RED,LEDConstants.WHITE,LEDConstants.FULL);
                Logger.info("INTKE", "Piece Missed");
            } else if(time.get() > 0.5 && shooting){
                Robot.lightStrip.section(3*LEDConstants.bufferSize/5,4*LEDConstants.bufferSize/5-1,LEDConstants.GREEN,LEDConstants.SATURATED,LEDConstants.FULL);
                Logger.info("INTKE", "Shot Fired");
            } else {
                Robot.lightStrip.section(3*LEDConstants.bufferSize/5,4*LEDConstants.bufferSize/5-1,Robot.lightStrip.alignOOB,LEDConstants.SATURATED,LEDConstants.FULL);
                Logger.info("INTKE", "Piece Acquired");
            }
            Robot.intake.setIntakeVoltage(0);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return  Robot.skipNonPath || 
                time.get() > runTime || 
                (time.get() > 0.5 && shooting)|| 
                (Robot.intake.getIntakeCurrent() > 24 && time.get() - intakeTime > 0.5);
    }
}