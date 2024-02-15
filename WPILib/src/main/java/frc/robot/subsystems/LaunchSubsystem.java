package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
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

public class LaunchSubsystem extends SubsystemBase {

    // https://www.revrobotics.com/rev-21-1650/
    private CANSparkMax GroundIntakeMotor, IntakeMotor, LaunchMotor;

    private SlewRateLimiter LaunchMotorRateLimiter =  new SlewRateLimiter(1, 1, 0);

    private PIDController GroundIntakePID = new PIDController(0.1, 0, 0.05);

    // private PIDController LaunchMotorPID = new PIDController(0, 0, 0);



    public double LaunchSpeed = 0;
    
    public double GroundIntakeSpeed = 0;

    public LaunchSubsystem() 
    {
        super();

        SmartDashboard.putData("Launch Ground PID", GroundIntakePID);

        if (RobotBase.isReal()) 
        {
            // this.LaunchMotor = new CANSparkMax(Constants.Launcher.CANPort, MotorType.kBrushless);
            this.GroundIntakeMotor = new CANSparkMax(Constants.GroundIntake.CANPort, MotorType.kBrushless);
            // this.IntakeMotor = new CANSparkMax(Constants.Launcher.IntakeMotor.CANPort, MotorType.kBrushless);

            // this.LaunchMotor.setInverted(false);
            // this.IntakeMotor.setInverted(false);
            this.GroundIntakeMotor.setInverted(true);

            // this.LaunchMotor.setIdleMode(IdleMode.kCoast);
            // this.IntakeMotor.setIdleMode(IdleMode.kCoast);
            this.GroundIntakeMotor.setIdleMode(IdleMode.kCoast);
        }

        setDefaultCommand(Commands.run(() -> 
        {
            this.GroundIntakeMotor.set(Controllers.DriverController.getYButton() ? 0.40 : 0);

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
        return 0;
        // return RobotBase.isReal() ? this.LaunchMotor.getEncoder().getVelocity() : 0;
    }

    public void Stop()
    {
        if (RobotBase.isReal())
        {
            this.GroundIntakeMotor.stopMotor();
            // this.LaunchMotor.stopMotor();
            // this.IntakeMotor.stopMotor();
        }
    }

    @Override
    public void periodic() 
    {
        if (RobotBase.isReal())
        {
            // this.GroundIntakeMotor.set(this.GroundIntakePID.calculate(this.GroundIntakeMotor.get(), this.GroundIntakeSpeed));
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
        return Commands.run(() -> this.LaunchSpeed = 1, this).onlyWhile(() -> this.LaunchMotor.get() != 1)
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
    }
}
