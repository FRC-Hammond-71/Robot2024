package frc.robot;

import java.util.MissingResourceException;

import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.utilities.DriverStationUtils;

public class FieldGeometry 
{
    public static Pose2d GetStartingPosition() throws Exception
    {
        Pose2d pose;

        switch (Robot.SPosition.getSelected())
        {
            case 1: pose = new Pose2d(15.82, 4.41, Rotation2d.fromDegrees(60)); break;
            case 2: pose = new Pose2d(15.151112, 5.524321, Rotation2d.fromDegrees(0)); break;
            case 3: pose = new Pose2d(15.80, 6.71, Rotation2d.fromDegrees(-60)); break;
            default: throw new Exception("Invalid Starting Position");
        }

        return DriverStation.getAlliance().get() == Alliance.Red ? pose : GeometryUtil.flipFieldPose(pose);
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
