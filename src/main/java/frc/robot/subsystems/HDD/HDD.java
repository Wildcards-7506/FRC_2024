package frc.robot.subsystems.HDD;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ControlConfigs.PlayerConfigs;
import frc.robot.ControlConfigs.Drivers.Ryan;
import frc.robot.ControlConfigs.Drivers.Anthony;
import frc.robot.ControlConfigs.Drivers.Ricardo;

public class HDD {      
    public static SendableChooser<PlayerConfigs> driver_chooser = new SendableChooser<>();
    public static SendableChooser<PlayerConfigs> coDriver_chooser = new SendableChooser<>();

    //Drivers options
    public static PlayerConfigs ryan = new Ryan();
    public static PlayerConfigs anthony = new Anthony();
    public static PlayerConfigs ricardo = new Ricardo();

    public static SequentialCommandGroup desiredMode;

    public static void initBot(){
        // Driver choosers
        driver_chooser.setDefaultOption("Ryan", ryan);
        driver_chooser.addOption("Anthony", anthony);
        driver_chooser.addOption("Ricardo", ricardo);        

        // Co-Driver choosers
        coDriver_chooser.setDefaultOption("Anthony", anthony);
        coDriver_chooser.addOption("Ricardo", ricardo);
        coDriver_chooser.addOption("Ryan", ryan);        

        // Put the choosers on the dashboard
        SmartDashboard.putData(driver_chooser);
        SmartDashboard.putData(coDriver_chooser);
    }
}