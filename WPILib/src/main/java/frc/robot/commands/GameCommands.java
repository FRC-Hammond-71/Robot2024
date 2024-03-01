package frc.robot.commands;

import java.time.Duration;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Launcher;
import frc.robot.Controllers;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.math.LauncherFiringSolution;
import frc.robot.utilities.Rotation2dUtils;

public class GameCommands
{
	public static Command IntakeNote()
	{
		return Robot.Arm.RunRotate(Constants.Arm.IntakeAngle)
			.andThen(Robot.Launcher.Intake())
			.onlyWhile(() -> Controllers.ShooterController.getRightBumper() && !Robot.Launcher.IsLoaded());
	}

	public static Command AutoRotateAndLaunch()
	{
		var firingSolution = LauncherFiringSolution.CalculateToSpeaker(Robot.Localization.GetEstimatedPose());

		System.out.printf("Firing at %.2f Degrees with %.2f Degrees of Error!\n", firingSolution.ArmAngle.getDegrees(), firingSolution.YawError.getDegrees());

		if (!Robot.Arm.InBounds(firingSolution.ArmAngle))
		{
			System.out.println("Cannot shoot...reached arm rotation boundary!");
			return ControllerCommands.RumbleController(Controllers.ShooterController, RumbleType.kBothRumble, 1, Duration.ofSeconds(1));
		}

		return new ParallelCommandGroup(
			new FaceAtCommand(firingSolution.TargetPosition.toTranslation2d(), firingSolution.YawError),
			Robot.Arm.RunRotate(firingSolution.ArmAngle))
			.andThen(Robot.Launcher.Launch(0.7, 0.7));
	}

	public static Command AutoPitchAndLaunch()
	{
		var firingSolution = LauncherFiringSolution.CalculateToSpeaker(Robot.Localization.GetEstimatedPose());

		System.out.printf("Firing at %.2f Degrees with %.2f Degrees of Error!\n", firingSolution.ArmAngle.getDegrees(), firingSolution.YawError.getDegrees());

		if (!Robot.Arm.InBounds(firingSolution.ArmAngle))
		{
			System.out.println("Cannot shoot...reached arm rotation boundary!");
			return ControllerCommands.RumbleController(Controllers.ShooterController, RumbleType.kBothRumble, 1, Duration.ofSeconds(1));
		}

		return Robot.Arm.RunRotate(firingSolution.ArmAngle).andThen(Robot.Launcher.Launch(0.7, 0.7));
	}

	public static Command GotoSpeakerAndLaunch()
	{
		return PathCommands.PathToSpeaker()
				.andThen(AutoRotateAndLaunch())
				.withName("GotoSpeakerAndLaunch");
	}
}
