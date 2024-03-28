package frc.robot.math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.FieldConstants;

public class SpeakerCalculations
{
    public class CurvedFiringSolution
    {

    }

    public static double CalculateDistanceFromSpeaker()
    {
        return Robot.Localization.GetEstimatedPose180().getTranslation().getDistance(FieldConstants.GetSpeakerPosition());
    }

    public static Rotation2d CalculatePitchToSpeaker()
    {
        double distanceFromSpeaker = CalculateDistanceFromSpeaker();
        
        // final double heightOfSpeakerIntake = Units.inchesToMeters(83);
        final double heightOfSpeakerIntake = Units.inchesToMeters(46);
        final double shooterYOffset = Units.feetToMeters(1.8);
        
        double rotationRadians = Math.atan((heightOfSpeakerIntake - shooterYOffset) / distanceFromSpeaker);

        return Rotation2d.fromRadians(rotationRadians);

        // var distanceFromSpeaker = CalculateDistanceFromSpeaker();
        // // Offset launcher in ft
        // var ys = 1.5;
        // // // Distance away in ft
        // var xs = Units.metersToFeet(distanceFromSpeaker);
        // // Range from 0-10 ("oomph" value)
        // var h = 0;

        // var vy = Math.sqrt(Math.pow(h, 2) + ((46 / 12) - ys) * 32 * 2);
        // var tm = (vy - h) / 35;
        // var vx = xs / tm;

        // // Launch angle (deg) 
        // var launchAngle = Rotation2d.fromDegrees(Math.tanh(vy / vx) * 180 / Math.PI);

        // return launchAngle;
    }

    public static double CalculateLaunchPercentageForSpeaker()
    {
        double distanceFromSpeaker = CalculateDistanceFromSpeaker();

        return Math.min(Math.max(Math.pow(distanceFromSpeaker / (2 / 3), 2) + 0.35, 0), 0.85);
    }
    
    public static double CalculateLaunchSpeedForSpeaker()
    {
        var distanceFromSpeaker = CalculateDistanceFromSpeaker();
        // Offset launcher in ft
        var ys = 1.5;
        // // Distance away in ft
        var xs = Units.metersToFeet(distanceFromSpeaker);
        // Range from 0-10 ("oomph" value)
        var h = 0;

        // final double heightOfSpeakerIntake = Units.feetToMeters(4);

        var vy = Math.sqrt(Math.pow(h, 2) + ((46 / 12) - ys) * 32 * 2);
        var tm = (vy - h) / 35;
        var vx = xs / tm;

        // Launch Speed (M/s)
        double launchSpeed = Units.feetToMeters(Math.sqrt(Math.pow(vy, 2) + Math.pow(vx, 2)));

        // Dampen launch speed!
        // launchSpeed = launchSpeed * (2d / 3d);

        return launchSpeed;
    }
}
