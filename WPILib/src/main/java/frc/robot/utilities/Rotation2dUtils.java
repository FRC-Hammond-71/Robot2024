package frc.robot.utilities;

import edu.wpi.first.math.geometry.Rotation2d;

public class Rotation2dUtils 
{
    public static boolean InBounds(Rotation2d rotation, Rotation2d lower, Rotation2d upper)
    {
        return rotation.getRotations() > lower.getRotations() && upper.getRotations() > rotation.getRotations();
    }
    public static boolean InBounds(Rotation2d rotation, Rotation2d bound)
    {
        return InBounds(rotation, bound.times(-1), bound);
    }

    public static Rotation2d Clamp(Rotation2d rotation2d, Rotation2d lower, Rotation2d upper)
    {
        return Rotation2d.fromRadians(
            Math.max(lower.getRadians(),
            Math.min(rotation2d.getRadians(), upper.getRadians())));
    }
}
