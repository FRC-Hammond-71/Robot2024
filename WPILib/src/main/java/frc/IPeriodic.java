package frc;

import java.time.Duration;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * Defines an object that has an extended periodic.
 */
public interface IPeriodic
{
    // TimedRobot.Callback is private! Womp womp.
    Runnable getCallback();
    Duration getCallbackPeriod();

    static void ApplyPeriodic(IPeriodic periodic, TimedRobot timedRobot)
    {
        timedRobot.addPeriodic(periodic.getCallback(), periodic.getCallbackPeriod().getSeconds());
    }
}
