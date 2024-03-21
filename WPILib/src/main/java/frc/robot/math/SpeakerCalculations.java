package frc.robot.math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.FieldConstants;

public class SpeakerCalculations
{
    public static double CalculateDistanceFromSpeaker()
    {
        return Robot.Localization.GetEstimatedPose180().getTranslation().getDistance(FieldConstants.GetSpeakerPosition());
    }

    public static Rotation2d CalculateAngleToSpeaker()
    {
        double distanceFromSpeaker = CalculateDistanceFromSpeaker();
        
        final double heightOfSpeakerIntake = Units.inchesToMeters(83);
        final double shooterYOffset = Units.feetToMeters(1.8);
        
        double rotationRadians = Math.atan((heightOfSpeakerIntake - shooterYOffset) / distanceFromSpeaker);

        return Rotation2d.fromRadians(rotationRadians);
    }

    public static double CalculateLaunchPercentageForSpeaker()
    {
        double distanceFromSpeaker = CalculateDistanceFromSpeaker();

        return Math.min(Math.max(Math.pow(distanceFromSpeaker / 5, 2) + 0.45, 0), 0.85);
    }
}
