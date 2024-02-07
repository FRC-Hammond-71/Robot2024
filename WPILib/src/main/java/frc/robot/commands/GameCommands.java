package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.LauncherFiringSolution;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FieldLocalizationSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class GameCommands 
{
    public static Command IntakeNoteAndLoadIntoLauncher(ArmSubsystem arm, LauncherSubsystem launch)
    {
        return arm.RunRotate(Constants.Arm.LoadingAngle)
            .andThen(Commands.parallel(launch.RunGroundIntake(), arm.RunIntake()))
            .onlyWhile(() -> !launch.IsLoaded());
    }

    public static Command AutoRotateAndLaunch()
    {
        var firingSolution = LauncherFiringSolution.CalculateToSpeaker(Robot.FieldLocalization.GetEstimatedPose());

        System.out.printf("Firing at %.2f Degrees\n", firingSolution.ArmAngle.getDegrees());

        // Consider checking if a note is loaded? If not, command will end?
        // Robot could roa
        return new FaceAtCommand(Robot.Drive, Robot.FieldLocalization, firingSolution.TargetPosition.toTranslation2d())
            .andThen(Robot.Arm.RunRotate(firingSolution.ArmAngle))
            .andThen(Robot.Launcher.Launch());
    }

    public static Command GotoSpeakerAndLaunch()
    {
        return PathCommands.PathToSpeaker().andThen(AutoRotateAndLaunch());
    }
}
