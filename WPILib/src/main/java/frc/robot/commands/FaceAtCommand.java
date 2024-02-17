package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LocalizationSubsystem;

public class FaceAtCommand extends Command {

	public final Translation2d Target;

	public PIDController OutputController = new PIDController(Math.PI * 1.5, Math.PI / 2, Math.PI / 4); 

	public FaceAtCommand(Translation2d position)
	{
		super();

		this.Target = position;

		addRequirements(RobotContainer.Drive);
		addRequirements(RobotContainer.Localization);
	}

	/**
	 * @return The angle between the Robot and the Target.
	 */
	public Rotation2d GetTargetHeading()
	{
		return Target.minus(RobotContainer.Localization.GetEstimatedPose().getTranslation()).getAngle();
	}

	public Rotation2d GetHeadingError()
	{
		return this.GetTargetHeading().minus(RobotContainer.Localization.GetEstimatedPose().getRotation());
	}

	@Override
	public void initialize() 
	{
		Constants.Field.getObject("Target").setPose(new Pose2d(this.Target, new Rotation2d()));	
	}

	@Override
	public boolean isFinished() 
	{
		var heading_error = this.GetHeadingError();
		// System.out.println(heading_error);
		return heading_error.getDegrees() < 1 && heading_error.getDegrees() > -1;
	}

	@Override
	public void execute()
	{		
		var output = this.OutputController.calculate(RobotContainer.Localization.GetEstimatedPose().getRotation().getRadians(), this.GetTargetHeading().getRadians());		
		// Hard-limit, just in case something goes wrong with the calculation! 
		output = Math.max(Math.min(Math.PI, output), -Math.PI);
		
		System.out.printf("Output: %.2f Current: %.2f Goal: %.2f\n", output, RobotContainer.Localization.GetEstimatedPose().getRotation().getRadians(), this.GetTargetHeading().getRadians());

		RobotContainer.Drive.Set(
			new ChassisSpeeds(0, 0, output));
	}

	@Override
	public void end(boolean interrupted) 
	{
		RobotContainer.Drive.Stop();
		// this.ProfiledPID.reset(0);
		System.out.println("We are done aligning!");
	}
}
