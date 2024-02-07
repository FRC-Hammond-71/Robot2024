package frc.robot.commands;

import com.revrobotics.CANSparkLowLevel.MotorType;

import java.time.Duration;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class RampLauncherCommand extends Command
{
    // https://www.revrobotics.com/rev-21-1651/
    private CANSparkMax LaunchMotor;

    private Timer StartedAt;

    public final Duration SpinUpDuration;

    public final double TargetSpeed;

    public RampLauncherCommand(Duration spinUpDuration, double speed)
    {
        if (RobotBase.isReal()) 
        {
            this.LaunchMotor = new CANSparkMax(Constants.Launcher.LaunchMotor.CANPort, MotorType.kBrushless);
        }

        this.SpinUpDuration = spinUpDuration;
        this.TargetSpeed = speed;
    }

    @Override
    public void initialize() 
    {
        this.StartedAt = new Timer();
    }

    @Override
    public void execute() 
    {
        var progression = (this.SpinUpDuration.toSeconds() - this.StartedAt.get()) / this.SpinUpDuration.toSeconds();

        if (RobotBase.isReal())
        {
            this.LaunchMotor.set(Math.min(Math.pow(progression, 2), 1));
        }
    }

    @Override
    public boolean isFinished() 
    {
        return RobotBase.isReal() ? this.LaunchMotor.get() >= this.TargetSpeed : this.StartedAt.hasElapsed(this.SpinUpDuration.getSeconds());
    }

    @Override
    public void end(boolean interrupted) 
    {
        this.LaunchMotor = null;
        // this.LaunchMotor.set(0);
    }
}
