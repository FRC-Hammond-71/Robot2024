package frc;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class ExtendedSubsystem extends SubsystemBase 
{
    public Optional<Command> getRequiringCommand()
    {
        return Optional.of(CommandScheduler.getInstance().requiring(this));
    }
}
