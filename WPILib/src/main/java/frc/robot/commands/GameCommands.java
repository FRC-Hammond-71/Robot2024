package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.LauncherFiringSolution;
import frc.robot.RobotContainer;

public class GameCommands 
{
    public static Command IntakeNoteAndLoadIntoLauncher()
    {
        return RobotContainer.Arm.RunRotate(Constants.Arm.LoadingAngle)
            .andThen(Commands.parallel(RobotContainer.Launcher.RunGroundIntake(), RobotContainer.Launcher.RunIntake()))
            .onlyWhile(() -> !RobotContainer.Launcher.IsLoaded())
            .withName("IntakeNoteAndLoadIntoLauncher");
    }

    public static Command AutoRotateAndLaunch()
    {
        var firingSolution = LauncherFiringSolution.CalculateToSpeaker(RobotContainer.Localization.GetEstimatedPose());

        System.out.printf("Firing at %.2f Degrees\n", firingSolution.ArmAngle.getDegrees());

        // Consider checking if a note is loaded? If not, command will end?
        return new FaceAtCommand(firingSolution.TargetPosition.toTranslation2d())
            .andThen(RobotContainer.Arm.RunRotate(firingSolution.ArmAngle))
            .andThen(RobotContainer.Launcher.Launch())
            .withName("AutoRotateAndLaunch");
    }

    public static Command GotoSpeakerAndLaunch()
    {
        return PathCommands.PathToSpeaker()
            .andThen(AutoRotateAndLaunch())
            .withName("GotoSpeakerAndLaunch");
    }
}
