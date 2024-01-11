// package frc.robot.commands;

// import java.text.DecimalFormat;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.ExampleArmConstants;
// import frc.robot.Robot;
// import frc.robot.ControlConfigs.PlayerConfigs;
// import frc.robot.util.Logger;

// public class ExampleTeleopCommand extends Command {
//     double armSetpoint;
//     private int prev_ArmState = -1;
//     private boolean latch = false;
//     private boolean release = false;
//     private static final DecimalFormat df = new DecimalFormat("0.00");

//     public ExampleTeleopCommand() {
//         addRequirements(Robot.exampleArm);
//     }

//     @Override
//     public void execute() {

//         //State Selection
//         if(Robot.controller0.getPOV() != prev_ArmState && !release){
//             latch = Robot.controller0.getPOV() == -1 ? true : false;
//             prev_ArmState = Robot.controller0.getPOV() != -1 ? Robot.controller0.getPOV() : prev_ArmState;
//         } else if (latch && Robot.controller0.getPOV() == prev_ArmState){
//             release = true;
//         } else if (latch && release){
//             prev_ArmState = -1;
//             release = false;
//         }

//         //Intake
//         if (PlayerConfigs.intake) {
//             Robot.exampleArm.setIntake(-8);
//         } else if (PlayerConfigs.release) {
//             Robot.exampleArm.setIntake(8);
//         } else {
//             Robot.exampleArm.setIntake(-1);
//         }

//         //Arm
//         // LOW
//         if (prev_ArmState == 0) {
//             armSetpoint = ExampleArmConstants.kArmLow;
//         // MID
//         } else if (prev_ArmState == 90){
//             armSetpoint = ExampleArmConstants.kArmLow;
//         // HIGH
//         } else if (prev_ArmState == 180) {
//             armSetpoint = ExampleArmConstants.kArmLow;
//         } else if(prev_ArmState == -1) {
//             armSetpoint = ExampleArmConstants.kArmClosed;
//         }

//         Robot.exampleArm.setArm(armSetpoint);

//         SmartDashboard.putNumber("Arm Setpoint", armSetpoint);

//         Logger.info("ARM", 
//         df.format(armSetpoint)
//         + " " + df.format(Robot.exampleArm.getArmEncoder())
//         + " " + df.format(Robot.exampleArm.getArmEncoder() - armSetpoint)
//         + " (Setpoint, Position, Error)");
        
//         Robot.exampleArm.errorCheck();
//     }
// }