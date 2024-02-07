package frc.robot.utilities;

import edu.wpi.first.math.geometry.Rotation2d;

public class Rotation2dUtils 
{
    public static boolean InBounds(Rotation2d rotation, Rotation2d lower, Rotation2d upper)
    {
        return rotation.getDegrees() > lower.getDegrees() && rotation.getDegrees() < upper.getDegrees();
    }
}
