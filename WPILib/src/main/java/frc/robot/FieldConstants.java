package frc.robot;

import java.util.MissingResourceException;
import java.util.Optional;

import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.utilities.DriverStationUtils;

public class FieldConstants 
{
    public static Optional<Pose2d> GetStartingPosition() 
    {
        Optional<Pose2d> pose;

        if (Robot.SPosition.getSelected() == null) return Optional.empty();

        switch (Robot.SPosition.getSelected())
        {
            case 1: pose = Optional.of(new Pose2d(15.82, 4.41, Rotation2d.fromDegrees(60))); break;
            case 2: pose = Optional.of(new Pose2d(15.151112, 5.524321, Rotation2d.fromDegrees(0))); break;
            case 3: pose = Optional.of(new Pose2d(15.80, 6.71, Rotation2d.fromDegrees(-60))); break;
            default: return Optional.empty();
        }

        return DriverStation.getAlliance().get() == Alliance.Red ? pose : Optional.of(GeometryUtil.flipFieldPose(pose.get()));
    }

    public static Translation2d GetSpeakerPosition()
    {
        final var position = new Translation2d(16.518642, 5.534473);

        DriverStationUtils.EnsureAllianceSelected();

        return DriverStation.getAlliance().get() == Alliance.Red ? position : GeometryUtil.flipFieldPosition(position);
    }

    public static Pose2d GetAmplifierPose() throws MissingResourceException
    {
        final var position = new Pose2d(1.860647, 7.76, Rotation2d.fromDegrees(90));

        DriverStationUtils.EnsureAllianceSelected();

        return DriverStation.getAlliance().get() == Alliance.Red ? GeometryUtil.flipFieldPose(position) : position;
    }
}
