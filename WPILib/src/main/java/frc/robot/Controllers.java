package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public final class Controllers 
{
    public static XboxController DriverController = new XboxController(Constants.Controllers.DriverPort);
    public static XboxController ShooterController = new XboxController(Constants.Controllers.ArmOperatorPort);    
}
