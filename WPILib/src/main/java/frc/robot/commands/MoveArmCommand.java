package frc.robot.commands;
import java.util.Set;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystem.ArmSubsystem;

public class MoveArmCommand extends CommandBase 
{
    private ArmSubsystem Arm;
    private double TargetRotation;
    // private double StartedAt;

    public MoveArmCommand(ArmSubsystem arm, double targetRotation)
    {
        this.Arm = arm;
        this.TargetRotation = targetRotation;
    }

    @Override
    public void initialize() 
    {
        // this.StartedAt = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() 
    {
        var rotation = this.Arm.GetArmRotation();
        var direction = this.TargetRotation - rotation > 0 ? 1 : -1;

        // NOTE: THIS NEEDS TO BE A COMMAND THAT WILL STOP THE MOTOR IF CANCELED (OR ELSE IT MAY NOT STOP).
        this.Arm.ArmMotor.set((direction * 20 * 60) / 5500);
    }

    @Override
    public boolean isFinished() 
    {
        return (this.Arm.GetArmRotation() - this.TargetRotation) < 10;
    }

    @Override
    public void end(boolean interrupted) 
    {
        this.Arm.ArmMotor.stopMotor();
    }
}
