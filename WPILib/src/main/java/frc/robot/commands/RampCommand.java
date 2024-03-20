package frc.robot.commands;

import java.util.function.Consumer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * A linear-function which increases over time using the given rate.
 */
public class RampCommand extends Command
{
    private double StartedAt = 0;

    private Consumer<Double> Output;
    private final double Max;
    private final double Rate;
    private double Initial;

    public double Current;

    public RampCommand(double initial, double max, double ratePerSec, Consumer<Double> output, Subsystem... requirements)
    {
        Rate = ratePerSec;        
        Initial = initial;
        Current = initial;
        Output = output;
        Max = max;

        addRequirements(requirements);
    }

    @Override
    public void initialize()
    {
        StartedAt = Timer.getFPGATimestamp();
    }

    @Override
    public void execute()
    {
        Current = Initial + Rate * (Timer.getFPGATimestamp() - StartedAt);
        // Clamp limit
        Current = Math.min(Current, Max);
        Output.accept(Current);
    }

    @Override
    public boolean isFinished()
    {
        return Current >= Max;
    }

    @Override
    public void end(boolean interrupted)
    {
        StartedAt = 0;
    }
}
