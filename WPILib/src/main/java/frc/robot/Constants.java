package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public final class Constants 
{
	public static Field2d Field = new Field2d();

	/**
	 * Weight of Robot in Lb
	 */
	public static double RobotWeight = 100;

	public static class RedField
	{
		/**
		 * The position of the note-entrance of the Speaker game element. 
		 */
		public static Translation3d SpeakerPosition = new Translation3d(16.194929, 5.517298, 0);
	}

	public static class Controllers
	{
		/**
		 * The driver port to be used when instantiating a controller.
		 */
		public static final int DriverPort = 0;

		/**
		 * The arm operator port to be used when instantiating a controller.
		 */
		public static final int ArmOperatorPort = 1;

		/**
		 * The required distance before any input will be registered.
		 */
		public static final double Deadzone = 0.08;
	}

	public static class Drivetrain 
	{
		// https://www.revrobotics.com/rev-21-1650/

		/**
		 * The maximum speed the robot may go forward per second.
		 */
		public static double MaxForwardSpeed = 6;

		/**
		 * The maximum rate-of-change of forward speeds in (M/s).
		 */
		public static double MaxForwardSpeedDelta = MaxForwardSpeed - 1;

		/**
		 * The maximum rotational speed the robot may go per second.
		 */
		public static Rotation2d MaxRotationalSpeed = Rotation2d.fromDegrees(250);

		/**
		 * The maximum rate-of-change of forward speeds in (M/s).
		 */
		public static double MaxRotationalSpeedDelta = Math.PI;

		/**
		 * Wheel radius in Meters.
		 */
		public static double WheelRadius = Units.inchesToMeters(2);

		/**
		 * Wheel circumference in Meters.
		 */
		// WheelRadius * 2 * Math.PI
		public static double WheelCircumference = WheelRadius * 2 * Math.PI;

		/**
		 * Wheels make 10.7 rotations per Motor rotation (10.71:1)
		 */
		public static double WheelGearing = 10.71;

		/**
		 * The track width in Meters.
		 */
		public static double TrackWidth = Units.inchesToMeters(21.487);

		/**
		 * The track circumference in Meters. 
		 */
		public static double TrackCircumference = TrackWidth * 2 * Math.PI;
	}

	public static class GroundIntake
	{
		public static class Motor
		{
			public static int Port = 5;
	
			public static double Speed = 0.8;
		}

	}

	public static class Arm
	{
		public static int PitchMotorCANPort = 8;

		public static int PitchEncoderChannel = 0;

		public static int RotationLimitSwitchChannel = 1;

		public static Rotation2d MaxAngle = Rotation2d.fromDegrees(110); 

		public static Rotation2d LoadingAngle = Rotation2d.fromDegrees(60); 

		// Shooting from further would require a 0.5 degree inaccuracy
		public static Rotation2d AllowedAngleError = Rotation2d.fromDegrees(1.5);

		public static double MomentOfInertia = 5;

		/**
		 * To rotate the Arm a full rotation, the motor must rotate 414.285714 times.
		 */
		public static double Gearing = 414.285714;

		/**
		 * The diameter of the chain plate that is connected to the Arm in Meters.
		 */
		public static double ChainDiameter = Units.inchesToMeters(6.211);

		public static double ChainCircumference = ChainDiameter * 2 * Math.PI;
	}

	public static class Launcher
	{
		/**
		 * The diameter of the Launchers wheel in Meters.
		 */
		public static double WheelDiameter = Units.inchesToMeters(3.965079); 

		public static class LaunchMotor
		{
			/**
			 * The desired CANSPARKMAX port of the Launch motor.
			 */
			public static int CANPort = 7;
		}

		public static class IntakeMotor
		{
			/**
			 * The desired CANSPARKMAX port of the Shooter Intake motor.
			 */
			public static int CANPort = 6;

			public static double Speed = 0.8;
		}
	}
}
