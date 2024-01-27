package frc.robot.subsystems.Movement;

import java.time.Duration;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.commands.ExtendedCommands;
import frc.robot.utilities.ChassisSpeedsUtils;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class ActualDriveSubsystem extends DriveSubsystem {

	// ------
	// Motors
	// ------
	private CANSparkMax LeftLeadMotor = new CANSparkMax(5, MotorType.kBrushless);
	private CANSparkMax RightLeadMotor = new CANSparkMax(4, MotorType.kBrushless);
	private CANSparkMax LeftFollowMotor = new CANSparkMax(3, MotorType.kBrushless);
	private CANSparkMax RightFollowMotor = new CANSparkMax(2, MotorType.kBrushless);
	// ---------
	// Motor PID (These need tuning!)
	// ---------
	private PIDController LeftMotorsPID = new PIDController(0.05, 0, 0);
	private PIDController RightMotorsPID = new PIDController(0.05, 0, 0);

	private SimpleMotorFeedforward FeedForward = new SimpleMotorFeedforward(0.10158, 2.161, 0.53799);

	// -----------
	// Controllers
	// -----------
	private DifferentialDriveKinematics Kinematics = new DifferentialDriveKinematics(
			Constants.Drivetrain.TrackWidth);

	private DifferentialDrivePoseEstimator PoseEstimator;

	private AHRS IMU = new AHRS(SPI.Port.kMXP);

	/**
	 * The desired speed and rotation the robot should be moving at.
	 */
	private ChassisSpeeds TargetSpeeds = new ChassisSpeeds();
	private DifferentialDriveWheelSpeeds PreviousWheelSpeeds = new DifferentialDriveWheelSpeeds();

	public ActualDriveSubsystem() 
	{
		super();

		this.LeftLeadMotor.setInverted(true);
		this.RightLeadMotor.setInverted(false);

		this.LeftFollowMotor.follow(this.LeftLeadMotor);
		this.RightFollowMotor.follow(this.RightLeadMotor);

		this.LeftLeadMotor.getEncoder().setPosition(0);
		this.RightLeadMotor.getEncoder().setPosition(0);

		this.PoseEstimator = new DifferentialDrivePoseEstimator(
				this.Kinematics,
				new Rotation2d(Units.degreesToRadians(-this.IMU.getAngle())),
				this.GetLeftWheelTravel(),
				this.GetRightWheelTravel(),
				new Pose2d(14.6, 5.64, Rotation2d.fromDegrees(0)),
				// LimelightHelpers.getBotPose2d("limelight").plus(new Transform2d(new Translation2d(16.54 / 2, 8.2 / 2), Rotation2d.fromDegrees(0))),
				VecBuilder.fill(0.02, 0.02, 0.01),
				VecBuilder.fill(0.05, 0.05, 0.05));

		AutoBuilder.configureRamsete(
				this::GetEstimatedPose,
				// NOTE: May not work?
				(pose) -> this.PoseEstimator.resetPosition(new Rotation2d(0), 0, 0, pose),
				this::GetChassisSpeeds,
				this::Drive,
				new ReplanningConfig(),
				() -> {
					// Boolean supplier that controls when the path will be mirrored for the red
					// alliance
					// This will flip the path being followed to the red side of the field.
					// THE ORIGIN WILL REMAIN ON THE BLUE SIDE

					var alliance = DriverStation.getAlliance();
					if (alliance.isPresent()) {
						return alliance.get() == DriverStation.Alliance.Red;
					}
					return false;
				},
				this);

		// #region Logging
		Shuffleboard.getTab("Movement").addNumber("Left Velocity Inaccuracy",
				() -> Math.abs(PreviousWheelSpeeds.leftMetersPerSecond - this.GetLeftWheelSpeed()));
		Shuffleboard.getTab("Movement").addNumber("Right Velocity Inaccuracy",
				() -> Math.abs(PreviousWheelSpeeds.rightMetersPerSecond - this.GetRightWheelSpeed()));

		Shuffleboard.getTab("Movement").addString("Desired Movement", () -> String.format("Forward: %.2f (M/s) Rot: %.2f (D/s)",
				this.TargetSpeeds.vxMetersPerSecond,
				Units.radiansToDegrees(this.TargetSpeeds.omegaRadiansPerSecond)));

		Shuffleboard.getTab("Movement").addString("Estimated Position", () -> {

			var estimatedPosition = this.GetEstimatedPose();

			return String.format("(%.2f, %.2f) %.2f°",
					estimatedPosition.getX(),
					estimatedPosition.getY(),
					estimatedPosition.getRotation().getDegrees());
		});

		Shuffleboard.getTab("Movement").addString("IMU", () -> String.format("Yaw: %.2f Pitch: %.2f Roll: %.2f",
				this.IMU.getAngle(),
				this.IMU.getPitch(),
				this.IMU.getRoll()));

		Shuffleboard.getTab("Movement").addNumber("Left Speed (M/s)", () -> GetLeftWheelSpeed());
		Shuffleboard.getTab("Movement").addNumber("Left Desired Speed (M/s)", () -> PreviousWheelSpeeds.leftMetersPerSecond);

		Shuffleboard.getTab("Movement").addNumber("Right Speed (M/s)", () -> GetRightWheelSpeed());
		Shuffleboard.getTab("Movement").addNumber("Right Desired Speed (M/s)", () -> PreviousWheelSpeeds.rightMetersPerSecond);

		Shuffleboard.getTab("Movement").add(this.LeftMotorsPID);
		Shuffleboard.getTab("Movement").add(this.RightMotorsPID);
		Shuffleboard.getTab("Movement").addDouble("Feed Forward",
				() -> this.FeedForward.calculate(this.TargetSpeeds.vxMetersPerSecond));

		// SmartDashboard.putNumber("PID LEFT ERROR", this.LeftMotorsPID.getPositionError());
		// SmartDashboard.putNumber("PID RIGHT ERROR", this.RightMotorsPID.getPositionError());
		// #endregion
	}

	// ------------------
	// Movement Reporting
	// ------------------
	/**
	 * @return The Left Motor Speed in Meters / Second.
	 */
	@Override
	public double GetLeftWheelSpeed() {
		return this.LeftLeadMotor.getEncoder().getVelocity() / 60 * Constants.Drivetrain.WheelCircumference
				/ Constants.Drivetrain.WheelGearing;
	}

	/**
	 * @return The Right Motor Speed in Meters / Second.
	 */
	@Override
	public double GetRightWheelSpeed() {
		return this.RightLeadMotor.getEncoder().getVelocity() / 60 * Constants.Drivetrain.WheelCircumference
				/ Constants.Drivetrain.WheelGearing;
	}

	public double GetLeftWheelTravel() {
		return this.LeftLeadMotor.getEncoder().getPosition()
				* Constants.Drivetrain.WheelCircumference
				/ Constants.Drivetrain.WheelGearing;
	}

	public double GetRightWheelTravel() {
		return this.RightLeadMotor.getEncoder().getPosition()
				* Constants.Drivetrain.WheelCircumference
				/ Constants.Drivetrain.WheelGearing;
	}

	/**
	 * @return The estimated position of the Robot in Meters.
	 */
	@Override
	public Pose2d GetEstimatedPose() {
		return this.PoseEstimator.getEstimatedPosition();
	}

	/**
	 * @return The Limelight Pose - Compensated for Limelight reporting with origin at middle of the map.
	 */
	private Pose2d GetLimelightPose()
	{
		var pose = LimelightHelpers.getBotPose2d("limelight");
		pose = new Pose2d(
			new Translation2d(pose.getX() + 16.54 / 2, pose.getY() + 8.2 / 2), 
			pose.getRotation());

		return pose;
	}

	@Override
	public void Drive(ChassisSpeeds speeds) {
		this.TargetSpeeds = speeds;
	}

	@Override
	public void Stop() {
		this.TargetSpeeds = new ChassisSpeeds();
		this.LeftLeadMotor.stopMotor();
		this.RightLeadMotor.stopMotor();
		// this.Drive.stopMotor();
	}

	@Override
	public void periodic() {

		this.TargetSpeeds = ChassisSpeedsUtils.Clamp(TargetSpeeds, Constants.Drivetrain.MaxForwardSpeed, 0, Constants.Drivetrain.MaxRotationalSpeed.getRadians());

		// Use kinematics to calculate desired wheel speeds.
		var desiredWheelSpeeds = this.Kinematics.toWheelSpeeds(this.TargetSpeeds);

		this.PreviousWheelSpeeds = desiredWheelSpeeds;
		
		this.PoseEstimator.update(
			new Rotation2d(Units.degreesToRadians(this.IMU.getAngle())),
			-this.GetLeftWheelTravel(),
			-this.GetRightWheelTravel());
			
		if (LimelightHelpers.getTV("limelight"))
		{
			// The Limelight has a target!
			this.PoseEstimator.addVisionMeasurement(
					GetLimelightPose(),
					LimelightHelpers.getLatency_Pipeline("limelight")
			);
		}
		else
		{
			// Rumble when loosing track of AprilTags. For fun! (Lol)
			ExtendedCommands.RumbleController(DriverController, RumbleType.kBothRumble, 0.5, Duration.ofMillis(500)).schedule();
		}

		this.Field.setRobotPose(this.GetEstimatedPose());

		this.LeftLeadMotor.setVoltage(
				LeftMotorsPID.calculate(this.GetLeftWheelSpeed(),
						desiredWheelSpeeds.leftMetersPerSecond)
						+ this.FeedForward.calculate(desiredWheelSpeeds.leftMetersPerSecond));
		this.RightLeadMotor.setVoltage(
				RightMotorsPID.calculate(this.GetRightWheelSpeed(),
						desiredWheelSpeeds.rightMetersPerSecond)
						+ this.FeedForward.calculate(desiredWheelSpeeds.rightMetersPerSecond));
	}

	@Override
	public ChassisSpeeds GetChassisSpeeds() {
		return this.TargetSpeeds;
	}
}