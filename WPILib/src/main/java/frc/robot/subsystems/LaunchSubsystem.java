package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.time.Duration;
import java.util.Optional;

import javax.swing.plaf.TreeUI;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.ColorSensorV3.ProximitySensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ProximitySensorResolution;

import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.KalmanFilterLatencyCompensator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.FunctionalPeriodic;
import frc.IPeriodic;
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

    private int NoteCount = 0;
    private boolean NoteLastDetected = false; 
    public boolean NoteDetected = false;
    //last note detection
    // private SlewRateLimiter LaunchMotorRateLimiter = new SlewRateLimiter(1, 1,
    // 0);

    // private PIDController LaunchMotorPID = new PIDController(0, 0, 0);

    // public IPeriodic NoteDetectionPeriodic;
    public Notifier NoteDetectorNotifier;

    public LaunchSubsystem(Robot robot)
    {
        super(robot);
    }

    @Override
    protected void initializeReal()
    {
        this.TopLaunchMotor = new CANSparkMax(8, MotorType.kBrushless);
        this.BottomLaunchMotor = new CANSparkMax(9, MotorType.kBrushless);

        this.GroundIntakeMotor = new CANSparkMax(Constants.GroundIntake.CANPort, MotorType.kBrushless);
        this.IntakeMotor = new CANSparkMax(7, MotorType.kBrushless);

        this.TopLaunchMotor.setInverted(false);
        this.BottomLaunchMotor.setInverted(false);
        this.GroundIntakeMotor.setInverted(true);
        this.IntakeMotor.setInverted(true);

        this.TopLaunchMotor.setIdleMode(IdleMode.kCoast);
        this.BottomLaunchMotor.setIdleMode(IdleMode.kCoast);
        this.IntakeMotor.setIdleMode(IdleMode.kCoast);
        this.GroundIntakeMotor.setIdleMode(IdleMode.kCoast);

        this.NoteSensor = new ColorSensorV3(Port.kOnboard);
        this.NoteSensor.configureProximitySensor(ProximitySensorResolution.kProxRes8bit, ProximitySensorMeasurementRate.kProxRate6ms);

        this.NoteDetectorNotifier = new Notifier(() -> 
        {
            boolean isDetected = this.NoteSensor.getProximity() > 100;

            SmartDashboard.putBoolean("Is Note Detected", isDetected);

            if (!NoteLastDetected && isDetected)
            {
                NoteCount += 1;
                NoteDetected = true;
            }
            else if (NoteCount >= 2 && !isDetected)
            {
                NoteDetected = false;
                NoteCount = 0;
            }
        
            NoteLastDetected = isDetected;
        });
        this.NoteDetectorNotifier.startPeriodic(0.006);
    }

    /**
     * @return Whether or not the launcher is loaded with a note.
     */
    public boolean IsLoaded()
    {
       return this.NoteDetected;
    }

    /**
     * Rotational speed of Launch motors in M/s
     * 
     * @return
     */
    public double Speed()
    {
        return RobotBase.isReal() ? this.TopLaunchMotor.getEncoder().getVelocity() / 60 * Constants.Launcher.WheelCircumference : 0;
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

    @Override
    protected void realPeriodic()
    {
        
    }


    public Command RunGroundIntake()
    {
        return Commands.runEnd(() -> this.GroundIntakeMotor.set(0.3), () -> this.GroundIntakeMotor.stopMotor(), this);
    }

    
    public Command RunIntake()
    {
        return Commands.runEnd(() -> this.IntakeMotor.set(0.3), () -> this.IntakeMotor.stopMotor(), this);
    }

    public Command Launch()
    {
        if (RobotBase.isSimulation())
            return Commands.none();

        // Wind-up, spin-up THEN start middle intake motors to push into launch motors
        // then END!
        // return Commands.run(() -> this.LaunchSpeed = 1, this).onlyWhile(() ->
        // this.TopLaunchMotor.get() != 1)
        // .andThen(Commands.runEnd(() -> this.IntakeMotor.set(0.3), () ->
        // this.IntakeMotor.stopMotor(), this))
        // .onlyWhile(() -> !this.IsLoaded())
        // .finallyDo(() -> {
        // this.LaunchSpeed = 0;
        // this.IntakeMotor.stopMotor();
        // });

        return Commands.none();
    }

    @Override
    public void initSendable(SendableBuilder builder)
    {
        builder.addDoubleProperty("Launcher Speed", () -> this.Speed(), null);
        // builder.addDoubleProperty("Intake Speed", () -> RobotBase.isReal() ?
        // this.GroundIntakeMotor.get() : 0, null);

        if (RobotBase.isReal())
        {
            // builder.addDoubleProperty("Note Proximity", () -> this.NoteSensor.getProximity(), null);
            
            builder.addBooleanProperty("Note Loaded", () -> this.IsLoaded(), null);
        }
    }

    public Command PerformSysID()
    {
        var topSysId = new SysIdRoutine(new SysIdRoutine.Config(
                Units.Volts.of(1).per(Units.Seconds.of(1)),
                Units.Volts.of(4),
                Units.Seconds.of(12)),
                new SysIdRoutine.Mechanism(
                    (voltage) -> this.TopLaunchMotor.setVoltage(voltage.baseUnitMagnitude()),
                    (log) ->
                    {
                        log.motor("flywheel-top")
                            .voltage(Units.Volts.of(this.TopLaunchMotor.getBusVoltage()))
                            .linearVelocity(Units.MetersPerSecond.of(this.TopLaunchMotor.getEncoder().getVelocity() / 60 * Constants.Launcher.WheelCircumference))
                            .linearPosition(Units.Meters.of(this.TopLaunchMotor.getEncoder().getPosition() * Constants.Launcher.WheelCircumference));
                    },
                    this));

        var bottomSysId = new SysIdRoutine(new SysIdRoutine.Config(
                Units.Volts.of(1).per(Units.Seconds.of(1)),
                Units.Volts.of(4),
                Units.Seconds.of(12)),
                new SysIdRoutine.Mechanism(
                    (voltage) -> this.BottomLaunchMotor.setVoltage(voltage.baseUnitMagnitude()),
                    (log) ->
                    {
                        log.motor("flywheel-bottom")
                            .voltage(Units.Volts.of(this.BottomLaunchMotor.getBusVoltage()))
                            .linearVelocity(Units.MetersPerSecond.of(this.BottomLaunchMotor.getEncoder().getVelocity() / 60 * Constants.Launcher.WheelCircumference))
                            .linearPosition(Units.Meters.of(this.BottomLaunchMotor.getEncoder().getPosition() * Constants.Launcher.WheelCircumference));
                    },
                    this));

        return topSysId.quasistatic(Direction.kForward)
            .andThen(topSysId.quasistatic(Direction.kReverse))
            .andThen(topSysId.dynamic(Direction.kForward))
            .andThen(topSysId.dynamic(Direction.kReverse))
            .finallyDo(() -> this.Stop())
            .andThen(new WaitCommand(10))
            .andThen(bottomSysId.quasistatic(Direction.kForward))
            .andThen(bottomSysId.quasistatic(Direction.kReverse))
            .andThen(bottomSysId.dynamic(Direction.kForward))
            .andThen(bottomSysId.dynamic(Direction.kReverse))
            .finallyDo(() -> this.Stop());
    }


}
