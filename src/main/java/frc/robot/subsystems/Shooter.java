package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.CANID;

public class Shooter extends SubsystemBase{

    private final CANSparkMax flywheelL;
    private final CANSparkMax flywheelR;

    public Shooter () {
        this.flywheelL = new CANSparkMax(CANID.FLYWHEEL_LEFT, MotorType.kBrushless);
        this.flywheelR = new CANSparkMax(CANID.FLYWHEEL_RIGHT, MotorType.kBrushless);
    }

    public void SetFlywheelVoltage(double volt) {
        flywheelL.setVoltage(volt);
        flywheelR.setVoltage(volt);
    }

}
