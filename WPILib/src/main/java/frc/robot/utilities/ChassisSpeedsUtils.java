package frc.robot.utilities;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ChassisSpeedsUtils 
{
    public static ChassisSpeeds Clamp(ChassisSpeeds speeds, double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond)
    {
        return new ChassisSpeeds(
            Math.max(Math.min(speeds.vxMetersPerSecond, vxMetersPerSecond), -vxMetersPerSecond),
            Math.max(Math.min(speeds.vyMetersPerSecond, vyMetersPerSecond), -vyMetersPerSecond),
            Math.max(Math.min(speeds.omegaRadiansPerSecond, omegaRadiansPerSecond), -omegaRadiansPerSecond)
        );
    }    
}
