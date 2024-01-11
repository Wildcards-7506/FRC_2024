package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Shooter {

    private CANSparkMax flywheelL;
    private CANSparkMax flywheelR;

    public Shooter (int flywheelL, int flywheelR) {
        this.flywheelL = new CANSparkMax(flywheelL, MotorType.kBrushless);
        this.flywheelR = new CANSparkMax(flywheelR, MotorType.kBrushless);
    }

    public void SetFlywheelVoltage(double volt) {
        flywheelL.setVoltage(volt);
        flywheelR.setVoltage(volt);
    }

}
