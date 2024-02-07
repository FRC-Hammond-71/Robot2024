package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Controllers;
import frc.robot.LauncherFiringSolution;
import frc.robot.Robot;
import frc.robot.commands.GameCommands;
import frc.robot.commands.PathCommands;

public class LauncherSubsystem extends SubsystemBase {

    // https://www.revrobotics.com/rev-21-1650/
    private CANSparkMax GroundIntakeMotor;
    
    // https://www.revrobotics.com/rev-21-1651/
    private CANSparkMax LaunchMotor;

    // private PIDController LaunchMotorPID = new PIDController(0, 0, 0);

    public LauncherSubsystem() 
    {
        if (RobotBase.isReal()) 
        {
            this.LaunchMotor = new CANSparkMax(Constants.Launcher.LaunchMotor.CANPort, MotorType.kBrushless);
            this.GroundIntakeMotor = new CANSparkMax(Constants.GroundIntake.Motor.Port, MotorType.kBrushless);
        } 
        else 
        {
            // // TODO: Support simulation of Robot
            // throw new UnsupportedOperationException("LauncherSubsystem cannot be simulated!");
        }

        setDefaultCommand(Commands.run(() -> 
        {
            // this.LaunchMotor.set(Controllers.ShooterController.getXButton() ? 1 : 0);

        }, this));
    }

    /**
     * @return Whether or not the launcher is loaded with a note.
     */
    public boolean IsLoaded() 
    {
        throw new UnsupportedOperationException("Note detection is not setup on the Launcher!");
    }

    public double Speed()
    {
        return RobotBase.isReal() ? this.LaunchMotor.getEncoder().getVelocity() : 0;
    }

    public void Stop()
    {
        if (RobotBase.isReal())
        {
            this.GroundIntakeMotor.stopMotor();
            this.LaunchMotor.stopMotor();
        }
    }
    
    @Override
    public void initSendable(SendableBuilder builder) 
    {
        builder.addDoubleProperty("Launcher Speed", () -> this.Speed(), null);
    }

    public Command RunGroundIntake()
    {
        return Commands.runEnd(() -> this.GroundIntakeMotor.set(1), () -> this.GroundIntakeMotor.stopMotor(), this);
    }

    public Command Launch()
    {
        return (RobotBase.isReal()
            ? Commands.runEnd(() -> this.LaunchMotor.set(1), () -> this.LaunchMotor.stopMotor(), this)
            : Commands.runEnd(() -> System.out.println("Going to launch!"), () -> System.out.println("Launched Note!"), this)).withTimeout(1);
        // return Commands.runEnd(() -> this.LaunchMotor.set(1), () -> this.LaunchMotor.stopMotor(), this).onlyWhile(this::IsLoaded);     
    }

}
