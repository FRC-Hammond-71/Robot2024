package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FieldLocalizationSubsystem;

public class FaceAtCommand extends Command {

	public Translation2d Position;

	private DriveSubsystem Drive;
	private FieldLocalizationSubsystem FieldLocalization;

	private PIDController TurningPID = new PIDController(0.5, 0.5, 0);

	public FaceAtCommand(DriveSubsystem drive, FieldLocalizationSubsystem localizationSubsystem, Translation2d position)
	{
		super();

		this.Drive = drive;
		this.FieldLocalization = localizationSubsystem;

		this.Position = position;

		addRequirements(drive);
		addRequirements(localizationSubsystem);
	}

	public Rotation2d GetTargetHeading()
	{
		return Position.minus(FieldLocalization.GetEstimatedPose().getTranslation()).getAngle().minus(Rotation2d.fromDegrees(180));
	}

	public Rotation2d GetHeadingError()
	{
		return this.GetTargetHeading().minus(FieldLocalization.GetEstimatedPose().getRotation());
	}

	@Override
	public void initialize() 
	{
		Constants.Field.getObject("Target").setPose(new Pose2d(this.Position, new Rotation2d()));	
	}

	@Override
	public boolean isFinished() 
	{
		var heading_error = this.GetHeadingError();
		System.out.println(heading_error);
		return heading_error.getDegrees() < 2 && heading_error.getDegrees() > -2;
	}

	@Override
	public void execute() 
	{		
		this.Drive.Set(
			new ChassisSpeeds(0, 0, TurningPID.calculate(FieldLocalization.GetEstimatedPose().getRotation().getRadians(), GetTargetHeading().getRadians())));
	}

	@Override
	public void end(boolean interrupted) 
	{
		this.Drive.Stop();
		this.TurningPID.reset();
		this.TurningPID.setSetpoint(0);
		System.out.println("We are done aligning with Speaker!");
	}
}
