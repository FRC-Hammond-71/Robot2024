package frc.robot.subsystems.Movement;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.controllers.PPRamseteController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.utilities.PathPlannerUtils;

public abstract class DriveSubsystem extends SubsystemBase {

    public Field2d Field = new Field2d();

    protected DriveSubsystem()
    {
        SmartDashboard.putData(Field);
    }

    public abstract double GetLeftWheelSpeed();
    public abstract double GetRightWheelSpeed();

    public abstract Pose2d GetEstimatedPose();

    public abstract ChassisSpeeds GetChassisSpeeds();

    public abstract void Drive(ChassisSpeeds speeds);

    public Command FollowPathByName(String pathName)
    {
        return this.FollowPath(PathPlannerPath.fromPathFile(pathName));
    }
    public Command FollowPath(PathPlannerPath path)
    {
        this.Field.getObject("traj").setTrajectory(PathPlannerUtils.TrajectoryToWPITrajectory(path.getTrajectory(this.GetChassisSpeeds(), this.GetEstimatedPose().getRotation())));

        return AutoBuilder.followPath(path);
        // return new FollowPathCommand(
        //     path, 
        //     this::GetEstimatedPose,
        //     this::GetChassisSpeeds, 
        //     this::Drive,
        //     new PPRamseteController(),
        //     // new PPLTVController(0.02),
        //     new ReplanningConfig(true, false),
        //     () -> {
        //         var alliance = DriverStation.getAlliance();
        //             if (alliance.isPresent()) {
        //                 return alliance.get() == DriverStation.Alliance.Red;
        //             }
        //             return false;
        //     });
    }

    public Command PathFindToPose(Pose2d pose, PathConstraints constraints)
    {
        return AutoBuilder.pathfindToPose(pose, constraints, 0);
    }

    public abstract void Stop();
    
}
