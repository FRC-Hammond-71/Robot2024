package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Movement.DriveSubsystem;

public class FaceAtCommand extends Command {

	public Pose2d Position;

	private DriveSubsystem Drive;

	private PIDController TurningPID = new PIDController(3, 2, 0);

	public FaceAtCommand(DriveSubsystem drive, Pose2d position)
	{
		super();

		this.Drive = drive;
		this.Position = position;

		addRequirements(drive);

		// Shuffleboard.getTab("Automation").add("Turning PID", this.TurningPID);
	}

	public Rotation2d GetTargetHeading()
	{
		return Position.getTranslation().minus(Drive.GetEstimatedPose().getTranslation()).getAngle().minus(Rotation2d.fromDegrees(180));
	}

	public Rotation2d GetHeadingError()
	{
		return this.GetTargetHeading().minus(Drive.GetEstimatedPose().getRotation());
	}

	@Override
	public void initialize() 
	{
		this.Drive.Field.getObject("Target").setPose(this.Position);	
	}

	@Override
	public boolean isFinished() 
	{
		var heading_error = this.GetHeadingError();
		System.out.println(heading_error);
		return heading_error.getDegrees() < 10 && heading_error.getDegrees() > -10;
	}

	@Override
	public void execute() 
	{		
		// this.Drive.Drive(new ChassisSpeeds(0, 0, this.GetHeadingError().getRadians() * 0.8 + Units.degreesToRadians(20)));
		this.Drive.Drive(new ChassisSpeeds(0, 0, TurningPID.calculate(this.Drive.GetEstimatedPose().getRotation().getRadians(), GetTargetHeading().getRadians())));
	}

	@Override
	public void end(boolean interrupted) 
	{
		// this.Drive.Field.getObject("Speaker").close();
		this.Drive.Stop();
		this.TurningPID.reset();
		this.TurningPID.setSetpoint(0);
	}
}
