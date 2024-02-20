package frc;

import java.time.Duration;

public class FunctionalPeriodic implements IPeriodic
{
    public final Runnable Callback;
    public final Duration Period;

    public FunctionalPeriodic(Runnable callback, Duration period)
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
    public Duration getCallbackPeriod()
    {
        return this.Period;
    }
    
}
