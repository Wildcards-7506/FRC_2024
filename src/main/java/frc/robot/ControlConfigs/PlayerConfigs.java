package frc.robot.ControlConfigs;

public class PlayerConfigs {
    //Constants
    public static double turnSpeed;
    public static double driveSpeed;
    public static double fineTurnSpeed;
    public static double fineDriveSpeed;
    
    //Drivetrain
    public static double xMovement;
    public static double yMovement;
    public static double turnMovement;
    public static boolean fineControlToggle;
    public static boolean snapUp;
    public static boolean snapRight;
    public static boolean snapDown;
    public static boolean snapLeft;
    public static boolean align;

    //Intake
    public static boolean ground;
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

    //Climbers
    public static boolean splitClimberControl;
    public static boolean climberLUp;
    public static boolean climberLDown;
    public static boolean climberRUp;
    public static boolean climberRDown;

    public void getDriverConfig(){}

    public void getoperatorConfig(){}
}