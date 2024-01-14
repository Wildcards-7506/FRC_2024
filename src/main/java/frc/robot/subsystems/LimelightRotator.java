package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANID;

public class LimelightRotator extends SubsystemBase{

    private SparkPIDController llPIDController;
    private RelativeEncoder llEncoder;
    private CANSparkMax llRotator;

    public LimelightRotator () {
        llRotator = new CANSparkMax(CANID.LL_ROTATOR, MotorType.kBrushless);

        llEncoder = llRotator.getEncoder();

        llPIDController = llRotator.getPIDController();
        llPIDController.setOutputRange(-1, 1);
        llRotator.setSmartCurrentLimit(Constants.limelightRotatorConstants.kRotateCurrentLimit);
        llPIDController.setP(Constants.limelightRotatorConstants.kRotatorKP);

        llRotator.burnFlash();
    }

    public void setLLRotatorPosition(double kPosition) {
        llPIDController.setReference(kPosition, ControlType.kPosition);
    }

    public double getPos() {
        return llEncoder.getPosition();
    }

}
