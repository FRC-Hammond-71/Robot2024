package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Controllers;
import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.subsystems.ArmPosition;

public class GameCommands
{
	public static Command IntakeNote()
	{
		// Need to fix (bad code)
		return Robot.Arm.RunUntilHolding(ArmPosition.Intake)
			.andThen(Robot.Launcher.AutoIntake())
			.onlyWhile(() -> DriverStation.isTeleop() ? true : Controllers.ShooterController.getAButton())
			.withName("Intake Note");
	}

	public static Command BeginTrackingSpeaker()
	{
		return Commands.runOnce(() -> Robot.Arm.Mode = ArmPosition.TrackingSpeaker);
	}

	public static Command ScoreAmp()
	{
		// Adjust these values to change amp note speed!
		return Robot.Arm.RunUntilHolding(ArmPosition.Amp).andThen(Robot.Launcher.Launch(0.23, 0.07));
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
			.andThen(Robot.Launcher.AutoLaunch())
			.withName("AutoPitchAndLaunch");
	}

	public static Command FaceAtSpeaker()
	{
		return new FaceAtCommand(FieldConstants.GetSpeakerPosition(), Rotation2d.fromDegrees(3));
	}

	public static Command AutoRotateAndLaunch()
	{
		// if (!Robot.Arm.InBounds(firingSolution.ArmAngle))
		// {
		// 	return ControllerCommands.RumbleController(Controllers.ShooterController, RumbleType.kBothRumble, 10, 0.5);
		// }

		return new ParallelCommandGroup(
				FaceAtSpeaker(),
				Robot.Arm.RunUntilHolding(ArmPosition.TrackingSpeaker))
				.andThen(Robot.Launcher.AutoLaunch())
				.withName("AutoRotateAndLaunch");
	}
}