package frc.robot.commands;

import java.time.Duration;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ControllerCommands 
{
    public static Command RumbleController(XboxController controller, RumbleType rumbleType, double intensity, Duration duration)
    {
        return Commands
            .runOnce(() -> controller.setRumble(rumbleType, intensity))
            .andThen(new WaitCommand(duration.getSeconds()))
            .finallyDo(() -> controller.setRumble(rumbleType, 0));
    }
}
