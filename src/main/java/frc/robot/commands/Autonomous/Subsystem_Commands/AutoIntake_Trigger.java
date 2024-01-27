package frc.robot.commands.Autonomous.Subsystem_Commands;

import frc.robot.Robot;
import frc.robot.util.Logger;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoIntake_Trigger extends Command{
    
    double runTime;
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
        time.reset();
        time.start();
    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Logger.info("INTKE", Double.toString(Robot.intake.getIntakeCurrent()) + " Amps");
        Robot.intake.setIntakeVoltage(12);
        Robot.intake.running = Robot.intake.getIntakeSpeed() > 200 ? true : false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if(time.get() > 3){
            Logger.info("INTKE", "Piece Missed");
        } else if(time.get()>0.5 && shooting){
            Logger.info("INTKE", "Shot Fired");
        } else {
            Logger.info("INTKE", "Piece Acquired");
        }
        Robot.intake.setIntakeVoltage(0);
        Robot.intake.running = false;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return  Robot.skipNonPath || time.get() > 3 || (time.get() > 0.5 && shooting)|| (Robot.intake.running && Robot.intake.getIntakeCurrent() > 20);
    }
}