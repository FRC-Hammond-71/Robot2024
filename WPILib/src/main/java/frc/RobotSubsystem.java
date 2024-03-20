package frc;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class RobotSubsystem<T> extends SubsystemBase
{
    public T Robot;

    protected RobotSubsystem(T robot)
    {
        super();

        this.Robot = robot;

        if (RobotBase.isReal())
        {
            this.initializeReal();
        }
        else 
        {
            this.initializeSimulated();
        }
    }

    protected void initializeReal() {}
    
    protected void initializeSimulated() {}

    @Override
    public void periodic()
    {
        if (RobotBase.isReal())
        {
            this.realPeriodic();
        }
    }

    protected void realPeriodic() {}
}
