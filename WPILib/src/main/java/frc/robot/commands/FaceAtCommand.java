package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.Robot;

public class FaceAtCommand extends Command
{
	public final Translation2d Point;
	public final Rotation2d AllowedError;

	private final PIDController RotationPID = new PIDController(0.08, 0, 0);

	public FaceAtCommand(Translation2d position, Rotation2d error)
	{
		super();

		this.Point = position;
		this.AllowedError = error;
		
		this.RotationPID.setTolerance(error.getDegrees());
		this.RotationPID.calculate(this.GetHeadingError().getDegrees());
		
		addRequirements(Robot.Drive);
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
	}

	@Override
	public boolean isFinished()
	{
		return this.RotationPID.atSetpoint();
	}

	@Override
	public void execute()
	{
		System.out.println(this.GetHeadingError().getDegrees());
		double output = -this.RotationPID.calculate(this.GetHeadingError().getDegrees(), 0);

		Robot.Drive.Set(new ChassisSpeeds(0, 0, output));
	}

	@Override
	public void end(boolean interrupted)
	{
		Robot.Drive.Stop();
		System.out.println("We are done aligning!");
	}
}
