package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AssistedNoteIntake 
{
    public static Rotation2d CalculateRotationSpeeds(Pose2d robotPose, double yaw, double distance)
    {
        // Apply deadzone of 2 degrees
        double yawCorrection = Math.abs(yaw) > 8 ?  Math.copySign(Math.pow(yaw / 4, 2), -yaw) : 0;

        yawCorrection *= 0.8;

        // Dampen yaw correction with formula
        // yawCorrection *= Math.copySign(Math.pow(Math.E, -yawCorrection / 5), yaw);

        // return new ChassisSpeeds(0, 0, yawCorrection);

        return Rotation2d.fromDegrees(yawCorrection);
    }
}
