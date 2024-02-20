package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LocalizationSubsystem;

public class FaceAtCommand extends Command
{
	public final Translation2d Target;

	public PIDController OutputController = new PIDController(Math.PI * 1.5, Math.PI / 2, Math.PI / 4);

	public final Rotation2d AllowedError;

	public FaceAtCommand(Translation2d position, Rotation2d error)
	{
		super();

		this.Target = position;
		this.AllowedError = error;

		addRequirements(Robot.Drive);
		addRequirements(Robot.Localization);
	}
	

	/**
	 * @return The angle between the Robot and the Target.
	 */
	public Rotation2d GetTargetHeading()
	{
		return Target.minus(Robot.Localization.GetEstimatedPose().getTranslation()).getAngle();
	}

	public Rotation2d GetHeadingError()
	{
		return this.GetTargetHeading().minus(Robot.Localization.GetEstimatedPose().getRotation());
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
		return heading_error.getDegrees() < AllowedError.getDegrees() && heading_error.getDegrees() > -AllowedError.getDegrees();
	}

	@Override
	public void execute()
	{
		var output = this.OutputController.calculate(
				Robot.Localization.GetEstimatedPose().getRotation().getRadians(),
				this.GetTargetHeading().getRadians());
		// Hard-limit, just in case something goes wrong with the calculation!
		output = Math.max(Math.min(Math.PI, output), -Math.PI);

		System.out.printf("Output: %.2f Current: %.2f Goal: %.2f\n", output,
				Robot.Localization.GetEstimatedPose().getRotation().getRadians(),
				this.GetTargetHeading().getRadians());

		Robot.Drive.Set(
				new ChassisSpeeds(0, 0, output));
	}

	@Override
	public void end(boolean interrupted)
	{
		Robot.Drive.Stop();
		// this.ProfiledPID.reset(0);
		System.out.println("We are done aligning!");
	}
}
