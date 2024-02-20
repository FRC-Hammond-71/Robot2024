package frc.robot.utilities;

import java.util.MissingResourceException;

import edu.wpi.first.wpilibj.DriverStation;

public class DriverStationUtils 
{
    public static void EnsureAllianceSelected() throws MissingResourceException
    {
        if (DriverStation.getAlliance().isEmpty())
        {
            throw new MissingResourceException("Driver Station must be connected and have an Alliance selected!", "DriverStation", "getAlliance");
        }
    }

}
