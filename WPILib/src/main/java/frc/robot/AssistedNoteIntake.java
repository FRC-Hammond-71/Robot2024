package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AssistedNoteIntake 
{
    public static ChassisSpeeds CalculateSpeeds(Pose2d robotPose, double yaw, double distance)
    {
        // Apply deadzone of 2 degrees
        double yawCorrection = Math.abs(yaw) > 2 ?  Math.copySign(Math.pow(yaw / 4, 2), -yaw) : 0;

        // Dampen yaw correction with formula
        yawCorrection *= Math.pow(Math.E, -yawCorrection / 5);

        return new ChassisSpeeds(0, 0, yawCorrection);
    }
}
