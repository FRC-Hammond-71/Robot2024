package frc.robot.subsystems.Movement;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindRamsete;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.controllers.PPRamseteController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.LayoutType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.commands.FaceAtCommand;
import frc.robot.utilities.PathPlannerUtils;

public abstract class DriveSubsystem extends SubsystemBase {

    public Field2d Field = new Field2d();

    public XboxController DriverController = new XboxController(Constants.Controllers.DriverPort);

    protected DriveSubsystem()
    {
        super();

        SmartDashboard.putData(Field);

        Shuffleboard.getTab("Automation").getLayout("Commands", BuiltInLayouts.kList).add("Red Speaker", new FaceAtCommand(this, new Pose2d(16.25, 5.6, Rotation2d.fromDegrees(0))));
        // Shuffleboard.getTab("Automation").getLayout("Commands", BuiltInLayouts.kList).add("Origin", new FaceAtCommand(this, new Pose2d(0, 0, Rotation2d.fromDegrees(0))));
        // Shuffleboard.getTab("Automation").add(new AlightWithSpeakerCommand(this)
        //     .raceWith(ExtendedCommands.Finished(() -> DriverController.getPOV(90) == 1)));

        setDefaultCommand(Commands.run(() -> {

            double forward = this.DriverController.getLeftY();
            forward = forward > -Constants.Controllers.Deadzone && forward < Constants.Controllers.Deadzone ? 0 : forward;

            double rotation = this.DriverController.getRightX();
            rotation = rotation > -Constants.Controllers.Deadzone && rotation < Constants.Controllers.Deadzone ? 0 : rotation;

            this.Drive(new ChassisSpeeds
            (
                forward * Constants.Drivetrain.MaxForwardSpeed,
                0,
                Constants.Drivetrain.MaxRotationalSpeed.times(rotation).getRadians()
            ));

        }, this));
    }

    public abstract double GetLeftWheelSpeed();

    public abstract double GetRightWheelSpeed();

    public abstract Pose2d GetEstimatedPose();

    public abstract ChassisSpeeds GetChassisSpeeds();

    public abstract void Drive(ChassisSpeeds speeds);

    public Command FollowPathByName(String pathName) {
        var command = this.FollowPath(PathPlannerPath.fromPathFile(pathName));
        command.setName(String.format("Follow %s", pathName));
        return command; 
    }

    public Command FollowPath(PathPlannerPath path) {
        this.Field.getObject("traj").setTrajectory(PathPlannerUtils.TrajectoryToWPITrajectory(
                path.getTrajectory(this.GetChassisSpeeds(), this.GetEstimatedPose().getRotation())));

        // return AutoBuilder.followPath(path);
        return new FollowPathCommand(
                path,
                this::GetEstimatedPose,
                this::GetChassisSpeeds,
                this::Drive,
                new PPRamseteController(),
                // new PPLTVController(0.02),
                new ReplanningConfig(false, false),
                () -> {
                    return false;
                    // var alliance = DriverStation.getAlliance();
                    // if (alliance.isPresent()) {
                    //     return alliance.get() == DriverStation.Alliance.Red;
                    // }
                    // return true;
                }, this);
    }

    public Command PathFindToPose(Pose2d pose, PathConstraints constraints) {

        return new PathfindRamsete(new Translation2d(pose.getX(), pose.getY()), 
                constraints,
                0,
                this::GetEstimatedPose,
                this::GetChassisSpeeds,
                this::Drive,
                1,
                1,
                new ReplanningConfig(false, false),
                this
        );
    }

    public abstract void Stop();

}
