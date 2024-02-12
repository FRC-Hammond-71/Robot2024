package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class PathUntilNoteCommand extends FollowPathCommand
{

    public final Supplier<Boolean> hasNote; 

    public PathUntilNoteCommand(
        PathPlannerPath path, Supplier<Pose2d> poseSupplier,
        Supplier<ChassisSpeeds> speedsSupplier,
        Supplier<Boolean> hasNote,
        Consumer<ChassisSpeeds> outputRobotRelative,
        PathFollowingController controller, ReplanningConfig replanningConfig, BooleanSupplier shouldFlipPath,
        Subsystem... requirements) 
    {
        super(path, poseSupplier, speedsSupplier, outputRobotRelative, controller, replanningConfig, shouldFlipPath,
                requirements);

        this.hasNote = hasNote;
    }

    @Override
    public void execute() 
    {
        // TODO: Possibly repath using vision data of note?
        super.execute();
    }

    @Override
    public boolean isFinished() 
    {
        // Continue path until Robot has note or path has been finished!
        return this.hasNote.get() || super.isFinished();
    }   
}
