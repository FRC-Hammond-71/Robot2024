package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.KalmanFilterLatencyCompensator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.RobotSubsystem;
import frc.robot.Constants;
import frc.robot.Controllers;
import frc.robot.Robot;
import frc.robot.commands.GameCommands;
import frc.robot.commands.PathCommands;
import frc.robot.math.LauncherFiringSolution;

public class LaunchSubsystem extends RobotSubsystem<Robot>
{

    // https://www.revrobotics.com/rev-21-1650/
    public CANSparkMax GroundIntakeMotor, IntakeMotor, TopLaunchMotor, BottomLaunchMotor;

    public ColorSensorV3 NoteSensor;

    // private SlewRateLimiter LaunchMotorRateLimiter =  new SlewRateLimiter(1, 1, 0);

    // private PIDController GroundIntakePID = new PIDController(0.1, 0, 0.05);

    // private PIDController LaunchMotorPID = new PIDController(0, 0, 0);

    public double LaunchSpeed = 0;
    public double GroundIntakeSpeed = 0;

    @Override
    protected void initializeReal()
    {
        this.TopLaunchMotor = new CANSparkMax(8, MotorType.kBrushless);
        this.BottomLaunchMotor = new CANSparkMax(9, MotorType.kBrushless);
        
        this.GroundIntakeMotor = new CANSparkMax(Constants.GroundIntake.CANPort, MotorType.kBrushless);
        this.IntakeMotor = new CANSparkMax(7, MotorType.kBrushless);
        
        this.TopLaunchMotor.setInverted(true);
        this.BottomLaunchMotor.setInverted(true);
        this.GroundIntakeMotor.setInverted(true);

        this.TopLaunchMotor.setIdleMode(IdleMode.kCoast);
        this.BottomLaunchMotor.setIdleMode(IdleMode.kCoast);
        this.IntakeMotor.setIdleMode(IdleMode.kCoast);
        this.GroundIntakeMotor.setIdleMode(IdleMode.kCoast);

        this.NoteSensor = new ColorSensorV3(Port.kOnboard);
    }

    public LaunchSubsystem(Robot robot) 
    {
        super(robot);
    }

    /**
     * @return Whether or not the launcher is loaded with a note.
     */
    public boolean IsLoaded() 
    {
        // throw new UnsupportedOperationException("Note detection is not setup on the Launcher!");

        if (RobotBase.isSimulation())
        {
            return false;
        }
        return this.NoteSensor.getProximity() > 1535.25;
    }

    public double Speed()
    {
        return 0;
        // return RobotBase.isReal() ? this.LaunchMotor.getEncoder().getVelocity() : 0;
    }

    public void SetLaunchSpeed(double percentage)
    {
        this.TopLaunchMotor.set(percentage);
        this.BottomLaunchMotor.set(percentage);
    }

    public void Stop()
    {
        if (RobotBase.isReal())
        {
            this.GroundIntakeMotor.stopMotor();
            this.IntakeMotor.stopMotor();
            this.TopLaunchMotor.stopMotor();
            this.BottomLaunchMotor.stopMotor();
        }
    }
    
    public Command RunGroundIntake()
    {
        return Commands.runEnd(() -> this.GroundIntakeMotor.set(1), () -> this.GroundIntakeMotor.stopMotor(), this);
    }
    
    public Command Launch()
    {
        if (RobotBase.isSimulation()) return Commands.none();
        
        // Wind-up, spin-up THEN start middle intake motors to push into launch motors then END!
        return Commands.run(() -> this.LaunchSpeed = 1, this).onlyWhile(() -> this.TopLaunchMotor.get() != 1)
            .andThen(Commands.runEnd(() -> this.IntakeMotor.set(0.3), () -> this.IntakeMotor.stopMotor(), this))
            .onlyWhile(() -> !this.IsLoaded())
            .finallyDo(() -> {
                this.LaunchSpeed = 0;
                this.IntakeMotor.stopMotor();
            });
    }

    public Command RunIntake()
    {
        return Commands.runEnd(() -> this.IntakeMotor.set(1), () -> this.IntakeMotor.stopMotor(), this);
    }
    
    @Override
    public void initSendable(SendableBuilder builder) 
    {
        builder.addDoubleProperty("Launcher Speed", () -> this.Speed(), null);
        builder.addDoubleProperty("Intake Speed", () -> RobotBase.isReal() ? this.GroundIntakeMotor.get() : 0, null);
        // builder.addDoubleProperty("Note Proximity", () -> this.NoteSensor.getProximity(), null);
    }
}
