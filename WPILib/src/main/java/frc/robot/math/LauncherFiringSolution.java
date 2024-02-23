package frc.robot.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.RedField;
import frc.robot.FieldGeometry;

public class LauncherFiringSolution implements Sendable
{
    public Rotation2d ArmAngle;
    /**
     * The required velocity in M/s.
     */
    public double LaunchVelocity;
    public Translation3d TargetPosition;
    public Rotation2d YawError;

    protected LauncherFiringSolution(Rotation2d angle, double speed, Translation3d target, Rotation2d yawError)
    {
        this.ArmAngle = angle;
        this.LaunchVelocity = speed;
        this.TargetPosition = target;
        this.YawError = yawError;
    }

    public static LauncherFiringSolution CalculateToSpeaker(Pose2d robotPose)
    {
        // The following calculations are credited to dydx#1394, and giant908. Thank you!
        // Equations from Desmos:  https://www.desmos.com/calculator/tsjv7opvqg

        var distanceFromSpeaker = robotPose.getTranslation().getDistance(Constants.RedField.SpeakerPosition.toTranslation2d());
        // Offset launcher in ft
        var ys = 0.75;
        // Distance away in ft
        var xs = Units.metersToFeet(distanceFromSpeaker);
        // Range from 0-10 ("oomph" value)
        var h = 5;
        var vy = Math.sqrt(Math.pow(h, 2) + ((80.5 / 12) - ys) * 32 * 2);
        var tm = (vy - h) / 32;
        var vx = xs / tm;

        // Launch angle (deg)
        var launchAngle = Rotation2d.fromDegrees(Math.tanh(vy / vx) * 180 / Math.PI);
        // Launch Speed (M/s)
        double launchSpeed = Units.feetToMeters(Math.sqrt(Math.pow(vy, 2) + Math.pow(vx, 2)));

        // var toLeft = FieldGeometry.GetSpeakerIntakeLeftMostPosition().minus(robotPose.getTranslation());
        // var toRight = FieldGeometry.GetSpeakerIntakeRightMostPosition().minus(robotPose.getTranslation());
        // var allowedError = Rotation2d.fromRadians(Math.abs(toLeft.getAngle().minus(toRight.getAngle()).getRadians()));
        var allowedError = Rotation2d.fromDegrees(1);

        // Use distance to magnify uncertainty
        
        return new LauncherFiringSolution(launchAngle, launchSpeed, Constants.RedField.SpeakerPosition, allowedError);
    }

    @Override
    public void initSendable(SendableBuilder builder) 
    {
        builder.publishConstDouble("Angle", this.ArmAngle.getDegrees());
        builder.publishConstDouble("Launch Velocity", this.LaunchVelocity);
        // builder.publishConstString("Target Position", this.TargetPosition.toString());
    }
}
