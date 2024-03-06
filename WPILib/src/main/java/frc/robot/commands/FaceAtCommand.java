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
	public final Translation2d Point;
	public final Rotation2d AllowedError;

	private final PIDController RotationPID = new PIDController(0.07, 0, 0.005);

	public FaceAtCommand(Translation2d position, Rotation2d error)
	{
		super();

		addRequirements(Robot.Drive);

		this.Point = position;
		this.AllowedError = error;
		
		this.RotationPID.setTolerance(0.5);
		this.RotationPID.calculate(this.GetHeadingError().getDegrees());
	}

	/**
	 * @return The angle between the Robot and the Target.
	 */
	public Rotation2d GetRequiredHeading()
	{
		return Point.minus(Robot.Localization.GetEstimatedPose().getTranslation()).getAngle();
	}

	public Rotation2d GetHeadingError()
	{
		return this.GetRequiredHeading().minus(Robot.Localization.GetEstimatedPose180().getRotation());
	}

	@Override
	public void initialize()
	{
		Constants.Field.getObject("Target").setPose(new Pose2d(this.Point, new Rotation2d()));

		// Constants.Field.getObject("Arrow").setPose(new Pose2d(Robot.Localization.GetEstimatedPose180().getTranslation(), this.GetHeadingError()));
	}

	@Override
	public boolean isFinished()
	{
		return this.RotationPID.atSetpoint();

		// var heading_error = this.GetHeadingError();

		// System.out.println(heading_error);
		// return heading_error.getDegrees() < AllowedError.getDegrees() && heading_error.getDegrees() > -AllowedError.getDegrees();
	}

	@Override
	public void execute()
	{
		System.out.println(this.GetHeadingError().getDegrees());
		double output = -this.RotationPID.calculate(this.GetHeadingError().getDegrees(), 0);

		// var output = Math.max(Math.min(Math.PI / 2, -this.GetHeadingError().getRadians() * 0.1), -Math.PI / 2);

		// System.out.printf("Output: %.2f Current: %.2f Goal: %.2f\n", output,
		// 		Robot.Localization.GetEstimatedPose().getRotation().getDegrees(),
		// 		this.GetTargetHeading().getDegrees());

		Robot.Drive.Set(
				new ChassisSpeeds(0, 0, output));
	}

	@Override
	public void end(boolean interrupted)
	{
		Robot.Drive.Stop();
		System.out.println("We are done aligning!");
	}
}
