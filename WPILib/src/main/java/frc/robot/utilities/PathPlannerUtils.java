package frc.robot.utilities;

import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory;

public final class PathPlannerUtils 
{
    public static Trajectory TrajectoryToWPITrajectory(PathPlannerTrajectory trajectory)
    {
        var states = trajectory.getStates()
            .stream()
            .map(s -> new Trajectory.State(s.timeSeconds, s.velocityMps, s.accelerationMpsSq, new Pose2d(s.positionMeters, new Rotation2d(0)), s.curvatureRadPerMeter))
            .toList();
        
        return new Trajectory(states);
    }
}
