package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.FieldGeometry;

public class PathCommands
{
    public static PathConstraints PathFindingConstraints = new PathConstraints(
            Constants.Drivetrain.MaxForwardSpeed,
            Constants.Drivetrain.MaxForwardSpeed,
            Constants.Drivetrain.MaxRotationalSpeed.getRadians(),
            Constants.Drivetrain.MaxRotationalSpeed.getRadians());

    public static Command PathToSpeaker()
    {
        return AutoBuilder.pathfindThenFollowPath(
                new PathPlannerPath(
                        PathPlannerPath.bezierFromPoses(
                                FieldGeometry.GetSpeakerShootingPose().plus(new Transform2d(-1, 0, new Rotation2d())),
                                FieldGeometry.GetSpeakerShootingPose()),
                        PathFindingConstraints, new GoalEndState(0, FieldGeometry.GetSpeakerShootingPose().getRotation())),
                PathFindingConstraints);
    }

    public static Command PathToAmplifier()
    {
        return AutoBuilder.pathfindThenFollowPath(
                new PathPlannerPath(
                        PathPlannerPath.bezierFromPoses(
                                FieldGeometry.GetAmplifierPose().plus(new Transform2d(-0.75, 0, new Rotation2d())),
                                FieldGeometry.GetAmplifierPose()),
                        PathFindingConstraints, new GoalEndState(0, FieldGeometry.GetAmplifierPose().getRotation())),
                PathFindingConstraints);
        // return AutoBuilder.pathfindToPose(Field.GetAllianceAmplifierPose().plus(new Transform2d(-2, 0, new Rotation2d())), PathFindingConstraints);
    }

//     public static Command PathToStage()
//     {
//         return AutoBuilder.pathfindThenFollowPath(
//                 new PathPlannerPath(
//                         PathPlannerPath.bezierFromPoses(
//                                 FieldGeometry.GetStageLineupPose(),
//                                 FieldGeometry.GetStagePose()),
//                         new PathConstraints(0.7, 2, Constants.Drivetrain.MaxRotationalSpeed.getRadians(), Constants.Drivetrain.MaxRotationalSpeed.getRadians()), new GoalEndState(0, FieldGeometry.GetStagePose().getRotation())),
//                 PathFindingConstraints);
//     }
}
