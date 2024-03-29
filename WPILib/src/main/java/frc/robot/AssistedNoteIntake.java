package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AssistedNoteIntake 
{
    public static Rotation2d CalculateRotationSpeeds(Pose2d robotPose, double yaw, double distance)
    {
        // Apply deadzone of 6 degrees
        double yawCorrection = Math.abs(yaw) > 6 ? yaw : 0;
        // double yawCorrection = Math.abs(yaw) > 6 ?  Math.copySign(Math.pow(yaw / 4, 2), -yaw) : 0;
        
        // Flipped parabola representing a percentage of correct to be applied (0 - 100%) 
        yawCorrection *= Math.max(-Math.pow(distance / 1.5, 2) + 1, 0);

        yawCorrection = -yawCorrection;

        return Rotation2d.fromDegrees(yawCorrection);
    }
}
