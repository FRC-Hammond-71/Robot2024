package frc.robot.commands;

import java.time.Duration;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Launcher;
import frc.robot.Controllers;
import frc.robot.FieldGeometry;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.math.LauncherFiringSolution;
import frc.robot.subsystems.ArmPosition;
import frc.robot.utilities.Rotation2dUtils;

public class GameCommands
{
	public static Command IntakeNote()
	{
		return Robot.Arm.RunUntilHolding(ArmPosition.Intake)
			.andThen(Robot.Launcher.RunIntake())
			.onlyWhile(() -> DriverStation.isTeleop() ? true : Controllers.ShooterController.getRightBumper() && !Robot.Launcher.IsLoaded())
			.withName("Intake Note");
	}

	public static Command BeginTrackingSpeaker()
	{
		return Commands.runOnce(() -> Robot.Arm.Mode = ArmPosition.TrackingSpeaker);
	}

	public static Command ScoreAmp()
	{
		return Robot.Arm.RunUntilHolding(ArmPosition.Amp).andThen(Robot.Launcher.RunLaunch(0.22, 0.05));
	}

	public static Command AutoPitch()
	{
		return Robot.Arm.RunUntilHolding(ArmPosition.TrackingSpeaker).withName("Auto Pitch");
	}

	public static Command AutoPitchAndLaunch()
	{
		// if (!Robot.Arm.InBounds(firingSolution.ArmAngle))
		// {
		// 	return ControllerCommands.RumbleController(Controllers.ShooterController, RumbleType.kBothRumble, 10, 0.5);
		// }

		return AutoPitch()
			.andThen(Robot.Launcher.RunLaunch(0.7, 0.7))
			.withName("AutoPitchAndLaunch");
	}

	public static Command AutoRotateAndLaunch()
	{
		// if (!Robot.Arm.InBounds(firingSolution.ArmAngle))
		// {
		// 	return ControllerCommands.RumbleController(Controllers.ShooterController, RumbleType.kBothRumble, 10, 0.5);
		// }

		return new ParallelCommandGroup(
				new FaceAtCommand(FieldGeometry.GetSpeakerPosition(), Rotation2d.fromDegrees(1.5)),
				Robot.Arm.RunUntilHolding(ArmPosition.TrackingSpeaker))
			.andThen(Robot.Launcher.RunLaunch(0.7, 0.7))
			.withName("AutoRotateAndLaunch");
	}
}
