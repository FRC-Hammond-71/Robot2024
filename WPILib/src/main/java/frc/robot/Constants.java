// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants 
{
	public static Field2d Field = new Field2d();

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

	public static class Drivetrain {
		// https://www.revrobotics.com/rev-21-1650/

		/**
		 * The maximum speed the robot may go forward per second.
		 */
		public static double MaxForwardSpeed = 4;

		/**
		 * The maximum rotational speed the robot may go per second.
		 */
		public static Rotation2d MaxRotationalSpeed = Rotation2d.fromDegrees(200);

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

	}

	public static class GroundIntake
	{
		public static class Motor
		{
			public static int Port = 5;
	
			public static double Speed = 0.8;
		}

	}

	public static class Launcher
	{
		public static int PitchMotorCANPort = 8;

		public static Rotation2d MaxPitch = Rotation2d.fromDegrees(80); 

		public static Rotation2d LoadingPitch = Rotation2d.fromDegrees(60); 
		
		public static Rotation2d MaximumLoadingPitchError = Rotation2d.fromDegrees(5); 

		public static int PitchEncoderChannel = 0;

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
	
		public static int RotationLimitSwitchChannel = 1;
	}
}
