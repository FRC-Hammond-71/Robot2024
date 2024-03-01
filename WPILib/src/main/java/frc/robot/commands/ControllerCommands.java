package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ControllerCommands
{
    public static Command RumbleController(XboxController controller, RumbleType rumbleType, double intensity,
            double durationSeconds)
    {
        return Commands
            .run(() -> controller.setRumble(rumbleType, intensity))
            .withTimeout(durationSeconds)
            .finallyDo(() -> controller.setRumble(rumbleType, 0));
    }
}
