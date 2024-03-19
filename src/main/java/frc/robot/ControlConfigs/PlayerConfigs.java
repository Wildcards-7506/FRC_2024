package frc.robot.ControlConfigs;

public class PlayerConfigs {
    //Constants
    public static double fullTurnSpeed;
    public static double fullDriveSpeed;
    public static double setupTurnSpeed;
    public static double setupDriveSpeed;
    public static double fineTurnSpeed;
    public static double fineDriveSpeed;
    
    //Drivetrain
    public static double xMovement;
    public static double yMovement;
    public static double turnMovement;
    public static boolean fineControlToggle;
    public static boolean setupControlToggle;
    public static boolean snapUp;
    public static boolean snapRight;
    public static boolean snapDown;
    public static boolean snapLeft;
    public static boolean zeroGyro;

    //Intake
    public static boolean amp;
    public static boolean stow;
    public static boolean fcEnable;
    public static double fcElbow;
    public static double fcWrist;

    //Shooter
    public static boolean armScoringMechanism;
    public static boolean shooterActive;
    public static boolean fire;
    public static boolean reject;
    public static boolean intake;

    //Climbers
    public static boolean climberEngage;

    public void getDriverConfig(){}

    public void getOperatorConfig(){}
}