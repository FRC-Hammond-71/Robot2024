package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Movement.DriveSubsystem;

public class AlightWithSpeakerCommand extends Command {

	// NOTE: This may not be correct.
	public static Pose2d SpeakerPosition = new Pose2d(16.25, 5.6, Rotation2d.fromDegrees(0));

	private DriveSubsystem Drive;

	private PIDController TurningPID = new PIDController(2, 0, 0);

	public AlightWithSpeakerCommand(DriveSubsystem drive)
	{
		super();

		this.Drive = drive;

		addRequirements(drive);

		Shuffleboard.getTab("Automation").add(this.TurningPID);
	}

	public Rotation2d GetTargetHeading()
	{
		return SpeakerPosition.getTranslation().minus(Drive.GetEstimatedPose().getTranslation()).getAngle();
	}

	public Rotation2d GetHeadingError()
	{
		return this.GetTargetHeading().minus(Drive.GetEstimatedPose().getRotation());
	}

	@Override
	public void initialize() 
	{
		this.Drive.Field.getObject("Speaker").setPose(SpeakerPosition);	
	}

	@Override
	public boolean isFinished() 
	{
		var heading_error = this.GetHeadingError();
		return heading_error.getDegrees() < 2 && heading_error.getDegrees() > -2;
	}

	@Override
	public void execute() 
	{		
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

	@Override
	public void initSendable(SendableBuilder builder) 
	{
		super.initSendable(builder);

		builder.setSmartDashboardType("");

		builder.addFloatProperty("Delta Heading Degree", () -> (float)GetHeadingError().getDegrees(), null);
	}
}
