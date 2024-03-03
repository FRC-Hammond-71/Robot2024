package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public final class Controllers 
{
    /**
     * The amplitude required before any input will be registered.
     */
    public static final double Deadzone = 0.08;

    public static XboxController DriverController = new XboxController(0);
    public static XboxController ShooterController = new XboxController(1);

    public static double ApplyDeadzone(double value)
    {
        return value > -Deadzone && value < Deadzone ? 0 : value;
    }

    public static double SquareInput(double value)
    {
        return Math.copySign(value * value, value);
    }
}
