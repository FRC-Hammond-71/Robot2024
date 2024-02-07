package frc.robot.commands;

import java.io.Console;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class PathCommands 
{
    public static PathConstraints PathFindingConstraints = new PathConstraints(
        Constants.Drivetrain.MaxForwardSpeed, 
        Constants.Drivetrain.MaxForwardSpeed, 
        Constants.Drivetrain.MaxRotationalSpeed.getRotations(), 
        Constants.Drivetrain.MaxRotationalSpeed.getRotations());

    public static Command PathToSpeaker()
    {
        return AutoBuilder.pathfindToPose(new Pose2d(14.751869, 5.557090, Rotation2d.fromDegrees(90)), PathFindingConstraints);
    }    
}
