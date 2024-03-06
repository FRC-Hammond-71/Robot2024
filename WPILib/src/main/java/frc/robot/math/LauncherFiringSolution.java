package frc.robot.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.FieldGeometry;

public class LauncherFiringSolution implements Sendable
{
    public Rotation2d ArmAngle;
    public double LaunchVelocity;
    public Translation3d TargetPosition;
    public Rotation2d AllowedYawError;

    protected LauncherFiringSolution(Rotation2d angle, double speed, Translation3d target, Rotation2d yawError)
    {
        this.ArmAngle = angle;
        this.LaunchVelocity = speed;
        this.TargetPosition = target;
        this.AllowedYawError = yawError;
    }

    public static LauncherFiringSolution CalculateToSpeaker(Pose2d robotPose)
    {
        double distanceFromSpeaker = robotPose.getTranslation().getDistance(FieldGeometry.GetSpeakerPosition());
        
        final double heightOfSpeakerIntake = Units.inchesToMeters(80.5);
        final double shooterYOffset = Units.feetToMeters(1.8);
        
        double rotationRadians = Math.atan((heightOfSpeakerIntake - shooterYOffset) / distanceFromSpeaker);

        return new LauncherFiringSolution(
            Rotation2d.fromRadians(rotationRadians), 
            0.7, 
            new Translation3d(0, 0, 0), 
            Rotation2d.fromDegrees(0.5));
    }

    @Override
    public void initSendable(SendableBuilder builder) 
    {
        builder.publishConstDouble("Angle", this.ArmAngle.getDegrees());
        builder.publishConstDouble("Launch Velocity", this.LaunchVelocity);
    }
}
