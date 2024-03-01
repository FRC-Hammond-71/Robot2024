package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

/**
 * A command which runs until a note is loaded into the launcher.
 */
public class UntilNoteLoadedCommand extends Command
{
    @Override
    public boolean isFinished()
    {
        return Robot.Launcher.IsLoaded();
    }
}
