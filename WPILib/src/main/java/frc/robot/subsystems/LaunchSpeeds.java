package frc.robot.subsystems;

public class LaunchSpeeds 
{
    public final double TopMetersPerSecond;
    public final double BottomMetersPerSecond;
    
    public LaunchSpeeds()
    {
        this(0, 0);
    }
    public LaunchSpeeds(double topMetersPerSecond, double bottomMetersPerSecond)
    {
        this.TopMetersPerSecond = topMetersPerSecond;
        this.BottomMetersPerSecond = bottomMetersPerSecond;
    }

    @Override
    public String toString()
    {
        return String.format("Top: %.2f Bottom: %.2f", this.TopMetersPerSecond, this.BottomMetersPerSecond);
    }
}
