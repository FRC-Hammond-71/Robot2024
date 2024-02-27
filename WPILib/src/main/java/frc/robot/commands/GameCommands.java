package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.math.LauncherFiringSolution;

public class GameCommands
{
	public static Command IntakeNote()
	{
		// return Robot.Arm.RunRotate(Constants.Arm.LoadingAngle)
		return Robot.Launcher.RunIntake()
			.onlyWhile(() -> !Robot.Launcher.IsLoaded())
			.withName("IntakeNote");
	}

	public static Command AutoRotateAndLaunch()
	{
		var firingSolution = LauncherFiringSolution.CalculateToSpeaker(Robot.Localization.GetEstimatedPose());

		System.out.printf("Firing at %.2f Degrees with %.2f Degrees of Error!\n", firingSolution.ArmAngle.getDegrees(), firingSolution.YawError.getDegrees());

		// Consider checking if a note is loaded? If not, command will end?
		// return new FaceAtCommand(firingSolution.TargetPosition.toTranslation2d(), firingSolution.YawError)
		// 		.andThen(Robot.Arm.RunRotate(firingSolution.ArmAngle))
		// 		.andThen(Robot.Launcher.Launch())
		// 		.withName("AutoRotateAndLaunch");
			
		return Robot.Arm.RunRotate(firingSolution.ArmAngle)
			.andThen(Robot.Launcher.Launch())
			.withName("PitchRotateAndLaunch");
	}

	public static Command GotoSpeakerAndLaunch()
	{
		return PathCommands.PathToSpeaker()
				.andThen(AutoRotateAndLaunch())
				.withName("GotoSpeakerAndLaunch");
	}
}
