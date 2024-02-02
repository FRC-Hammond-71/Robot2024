package frc.robot.commands;

import java.time.Duration;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class UtilCommands 
{
    public static Command Finished(BooleanSupplier condition, Subsystem... requirements)
    {
        return new FunctionalCommand(
            () -> {},
            () -> {},
            (interrupted) -> {},
            condition, 
            requirements);
    }

    public static Command RumbleController(XboxController controller, RumbleType rumbleType, double intensity, Duration duration)
    {
        return Commands
            .runOnce(() -> controller.setRumble(rumbleType, intensity))
            .andThen(new WaitCommand(duration.getSeconds()))
            .finallyDo(() -> controller.setRumble(rumbleType, 0));
    }
}
