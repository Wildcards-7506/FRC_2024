package frc.robot.commands.Autonomous.Subsystem_Commands;

import frc.robot.Constants.LEDConstants;
import frc.robot.Robot;
import frc.robot.util.Logger;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoShooterSpinUp extends Command{
    
    double lSpeed;
    double rSpeed;

    /** Creates a new Drivetrain Snap-to-angle Command. */
    public AutoShooterSpinUp(double lSpeed, double rSpeed) {
        this.lSpeed = lSpeed;
        this.rSpeed = rSpeed;
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
            Logger.info("SHOOT", "Right: " + Double.toString(Robot.shooter.getRSpeed()) + " RPM, " + "Left: " + Double.toString(Robot.shooter.getLSpeed()) + " RPM");
            Robot.shooter.setShooterSpeed(lSpeed,rSpeed);
            System.out.print("RIGHT: ");
            System.out.print(Robot.shooter.getRSpeed());
            System.out.print(" LEFT: ");
            System.out.println(Robot.shooter.getRSpeed());
            Robot.lightStrip.section(0,LEDConstants.bufferSize/5-1,Robot.lightStrip.shooterLo,LEDConstants.SATURATED,LEDConstants.FULL);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Logger.info("SHOOT", "Shooter Up To Speed");
        Robot.lightStrip.section(0,LEDConstants.bufferSize/5-1,LEDConstants.GREEN,LEDConstants.SATURATED,LEDConstants.FULL);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return  Robot.skipNonPath ||  Robot.shooter.getLSpeed() > 4000 && Robot.shooter.getRSpeed() < -4000;
    }
}