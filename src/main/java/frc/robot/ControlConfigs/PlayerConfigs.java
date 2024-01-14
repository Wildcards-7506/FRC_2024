package frc.robot.ControlConfigs;

public class PlayerConfigs {

    //buttons used: left joystick, right joystick, circle, entire d pad, L1, L2, R1, R2
    
    //primary drivetrain controls
    public static double xMovement;
    public static double yMovement;
    public static double turnMovement;
    public static double turnSpeed;
    public static double driveSpeed;
    public static double fineTurnSpeed;
    public static double fineDriveSpeed;
    public static boolean fineControlToggle;
    public static boolean xToggle;
    public static boolean snapZero;
    public static boolean snap90;
    public static boolean snap180;
    public static boolean snap270;
    public static boolean align;
    public static boolean brake;

    //arm
    public static int armControl;
    public static double armPos;
    public static boolean armFineControl;

    //limelight
    public static boolean switchPipeline;

    //Intake
    public static boolean intake;
    public static boolean release;

    //Shooter
    public static boolean shooter;
    public static boolean shooterPrime;

    //climbers
    public static boolean climberup;
    public static boolean climberdown;

    public void getDriverConfig(){}

    public void getCoDriverConfig(){}
}