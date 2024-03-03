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
        var location = Robot.SPosition.getSelected();

        if (location == 3)
        {
            return GetStartingPosition3();
        }
        else if (location == 2)
        {
            return GetStartingPosition2();
        }
        else if (location == 1)
        {
            return GetStartingPosition1();
        }
        throw new Exception("Invalid Starting Position");
    }

    public static Pose2d GetStartingPosition3()
    {
        final var pose = new Pose2d(15.80, 6.71, Rotation2d.fromDegrees(-60));

        DriverStationUtils.EnsureAllianceSelected();

        return DriverStation.getAlliance().get() == Alliance.Red ? pose : GeometryUtil.flipFieldPose(pose);
    }

    public static Pose2d GetStartingPosition2()
    {
        final var pose = new Pose2d(15.151112, 5.653389, Rotation2d.fromDegrees(0));

        DriverStationUtils.EnsureAllianceSelected();

        return DriverStation.getAlliance().get() == Alliance.Red ? pose : GeometryUtil.flipFieldPose(pose);
    }

        public static Pose2d GetStartingPosition1()
    {
        final var pose = new Pose2d(15.82, 4.41, Rotation2d.fromDegrees(60));

        DriverStationUtils.EnsureAllianceSelected();

        return DriverStation.getAlliance().get() == Alliance.Red ? pose : GeometryUtil.flipFieldPose(pose);
    }

    public static Pose2d GetSpeakerShootingPose() throws MissingResourceException
    {
        final var pose = new Pose2d(1.87, 5.58, Rotation2d.fromDegrees(180));

        DriverStationUtils.EnsureAllianceSelected();

        return DriverStation.getAlliance().get() == Alliance.Red ? GeometryUtil.flipFieldPose(pose) : pose;
    }

    public static Translation2d GetSpeakerIntakePosition()
    {
        final var position = new Translation2d(16.518642, 5.534473);

        DriverStationUtils.EnsureAllianceSelected();

        return DriverStation.getAlliance().get() == Alliance.Red ? position : GeometryUtil.flipFieldPosition(position);
    }
    public static Translation2d GetSpeakerIntakeLeftMostPosition() throws MissingResourceException
    {
        // TODO: UPDATE 0.3556 TO FLIP WHEN ON RED
        final var position = new Translation2d(0.427861, 4.971526 + 0.381);

        DriverStationUtils.EnsureAllianceSelected();

        return DriverStation.getAlliance().get() == Alliance.Red ? GeometryUtil.flipFieldPosition(position) : position;
    }
    public static Translation2d GetSpeakerIntakeRightMostPosition() throws MissingResourceException
    {
        // TODO: UPDATE 0.3556 TO FLIP WHEN ON RED
        final var position = new Translation2d(0.427861, 6.142910 - 0.381);

        DriverStationUtils.EnsureAllianceSelected();

        return DriverStation.getAlliance().get() == Alliance.Red ? GeometryUtil.flipFieldPosition(position) : position;
    }

    public static Pose2d GetAmplifierPose() throws MissingResourceException
    {
        final var position = new Pose2d(1.860647, 7.76, Rotation2d.fromDegrees(90));

        DriverStationUtils.EnsureAllianceSelected();

        return DriverStation.getAlliance().get() == Alliance.Red ? GeometryUtil.flipFieldPose(position) : position;
    }

    public static Pose2d GetStagePose() throws MissingResourceException
    {
        final var position = new Pose2d(4.341105, 4.958724, Rotation2d.fromDegrees(-60));

        DriverStationUtils.EnsureAllianceSelected();

        return DriverStation.getAlliance().get() == Alliance.Red ? GeometryUtil.flipFieldPose(position) : position;
    }
    public static Pose2d GetStageLineupPose() throws MissingResourceException
    {
        final var position = new Pose2d(3.994252, 5.770045, Rotation2d.fromDegrees(-60));

        DriverStationUtils.EnsureAllianceSelected();

        return DriverStation.getAlliance().get() == Alliance.Red ? GeometryUtil.flipFieldPose(position) : position;
    }
}
