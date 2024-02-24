package frc;

import java.time.Duration;

public class FunctionalPeriodic implements IPeriodic
{
    public final Runnable Callback;
    public final double Period;

    public FunctionalPeriodic(Runnable callback, double period)
    {
        this.Callback = callback;
        this.Period = period;
    }

    @Override
    public Runnable getCallback()
    {
        return this.Callback;
    }

    @Override
    public double getCallbackPeriod()
    {
        return this.Period;
    }
    
}
