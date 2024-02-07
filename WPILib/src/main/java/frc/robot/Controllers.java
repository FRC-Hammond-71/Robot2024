package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public final class Controllers 
{
    public static XboxController DriverController = new XboxController(Constants.Controllers.DriverPort);
    public static XboxController ShooterController = new XboxController(Constants.Controllers.ArmOperatorPort);

    public static double ApplyDeadzone(double value)
    {
        return value > -Constants.Controllers.Deadzone && value < Constants.Controllers.Deadzone ? 0 : value;
    }
}
