package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.I2C.Port;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.ColorSensorV3.ProximitySensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ProximitySensorResolution;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.RobotSubsystem;
import frc.robot.Constants;
import frc.robot.Robot;

public class LaunchSubsystem extends RobotSubsystem<Robot>
{
    /**
     * Any value larger than this threshold from the sensor will mark a detected note. 
     */
    public static final int NoteProximityThreshold = 100;

    // https://www.revrobotics.com/rev-21-1650/
    public CANSparkMax GroundIntakeMotor, IntakeMotor, TopLaunchMotor, BottomLaunchMotor;

    // public SimpleMotorFeedforward FeedForward = new SimpleMotorFeedforward(0.05, 0.2);

    public ColorSensorV3 NoteSensor;

    public LaunchSpeeds TargetSpeeds = new LaunchSpeeds();

    public LaunchSubsystem(Robot robot)
    {
        super(robot);
    }

    @Override
    protected void initializeReal()
    {
        this.TopLaunchMotor = new CANSparkMax(8, MotorType.kBrushless);
        this.BottomLaunchMotor = new CANSparkMax(9, MotorType.kBrushless);

        this.TopLaunchMotor.getEncoder().setPositionConversionFactor(Constants.Launcher.WheelCircumference);
        this.TopLaunchMotor.getEncoder().setVelocityConversionFactor(Constants.Launcher.WheelCircumference);
        this.BottomLaunchMotor.getEncoder().setPositionConversionFactor(Constants.Launcher.WheelCircumference);
        this.BottomLaunchMotor.getEncoder().setVelocityConversionFactor(Constants.Launcher.WheelCircumference);

        this.GroundIntakeMotor = new CANSparkMax(Constants.GroundIntake.CANPort, MotorType.kBrushless);
        this.IntakeMotor = new CANSparkMax(7, MotorType.kBrushless);

        this.TopLaunchMotor.setInverted(false);
        this.BottomLaunchMotor.setInverted(false);
        this.GroundIntakeMotor.setInverted(true);
        this.IntakeMotor.setInverted(true);

        this.TopLaunchMotor.setIdleMode(IdleMode.kCoast);
        this.BottomLaunchMotor.setIdleMode(IdleMode.kCoast);
        this.IntakeMotor.setIdleMode(IdleMode.kBrake);
        this.GroundIntakeMotor.setIdleMode(IdleMode.kCoast);

        this.NoteSensor = new ColorSensorV3(Port.kOnboard);
        this.NoteSensor.configureProximitySensor(ProximitySensorResolution.kProxRes11bit, ProximitySensorMeasurementRate.kProxRate6ms);
    }

    /**
     * @return Whether or not the launcher is loaded with a note.
     */
    public boolean IsLoaded()
    {
        return RobotBase.isReal() ? this.NoteSensor.getProximity() > NoteProximityThreshold : false;
    }

    public LaunchSpeeds GetSpeeds()
    {
        if (RobotBase.isSimulation()) return new LaunchSpeeds();

        return new LaunchSpeeds(
            this.TopLaunchMotor.getEncoder().getVelocity() / 60 * Constants.Launcher.WheelCircumference,
            this.BottomLaunchMotor.getEncoder().getVelocity() / 60 * Constants.Launcher.WheelCircumference
        );
    }

    public void SetLaunchSpeed(double percentage)
    {
        if (RobotBase.isSimulation()) return;

        this.TopLaunchMotor.set(percentage);
        this.BottomLaunchMotor.set(percentage);
    }
    public void SetLaunchSpeed(double percentageTop, double percentageBottom)
    {
        if (RobotBase.isSimulation()) return;

        this.TopLaunchMotor.set(percentageTop);
        this.BottomLaunchMotor.set(percentageBottom);
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

    public Command Intake()
    {
        if (RobotBase.isSimulation()) return Commands.none();

        return this.runEnd(() -> { this.IntakeMotor.set(0.3); this.GroundIntakeMotor.set(0.6); }, () -> { this.IntakeMotor.stopMotor(); this.GroundIntakeMotor.stopMotor(); })
            .withName("Intake");
    }

    public Command RunLaunch(double percentageTop, double percentageBottom)
    {
        if (RobotBase.isSimulation()) return Commands.none();

        return this.runOnce(() -> this.SetLaunchSpeed(percentageTop, percentageBottom))
            .andThen(new WaitCommand(0.5))
            .andThen(this.runOnce(() -> this.IntakeMotor.set(0.3)))
            .until(() -> !this.IsLoaded())
            .andThen(Commands.waitSeconds(0.1))
            .finallyDo(() -> this.Stop())
            .withName("Launch Note");
    }

    @Override
    public void initSendable(SendableBuilder builder)
    {
        builder.addDoubleProperty("Top Launcher Speed", () -> this.GetSpeeds().TopMetersPerSecond, null);
        builder.addDoubleProperty("Bottom Launcher Speed", () -> this.GetSpeeds().BottomMetersPerSecond, null);
        builder.addBooleanProperty("Note Loaded", () -> this.IsLoaded(), null);

        if (RobotBase.isReal())
        {
            builder.addDoubleProperty("Note Proximity", () -> this.NoteSensor.getProximity(), null);
        }
    }

    public Command PerformSysID()
    {
        var topSysId = new SysIdRoutine(new SysIdRoutine.Config(
                Units.Volts.of(1).per(Units.Seconds.of(1)),
                Units.Volts.of(8),
                Units.Seconds.of(6)),
                new SysIdRoutine.Mechanism(
                    (voltage) ->
                    {
                         this.TopLaunchMotor.setVoltage(voltage.magnitude());
                    },
                    (log) ->
                    {
                        var wheelVelocity = this.GetSpeeds();

                        log.motor("launch-flywheel-top")
                            .voltage(Units.Volts.of(this.TopLaunchMotor.getBusVoltage()))
                            .linearVelocity(Units.MetersPerSecond.of(wheelVelocity.TopMetersPerSecond))
                            .linearPosition(Units.Meters.of(this.TopLaunchMotor.getEncoder().getPosition() * Constants.Launcher.WheelCircumference));
                    },
                    this));

        var bottomSysId = new SysIdRoutine(new SysIdRoutine.Config(
                Units.Volts.of(1).per(Units.Seconds.of(1)),
                Units.Volts.of(8),
                Units.Seconds.of(6)),
                new SysIdRoutine.Mechanism(
                    (voltage) -> 
                    {
                        System.out.println(voltage);
                        this.BottomLaunchMotor.setVoltage(voltage.magnitude());
                    },
                    (log) ->
                    {
                        var wheelVelocity = this.GetSpeeds();

                        log.motor("launch-flywheel-bottom")
                            .voltage(Units.Volts.of(this.BottomLaunchMotor.getBusVoltage()))
                            .linearVelocity(Units.MetersPerSecond.of(wheelVelocity.BottomMetersPerSecond))
                            .linearPosition(Units.Meters.of(this.BottomLaunchMotor.getEncoder().getPosition() * Constants.Launcher.WheelCircumference));
                    },
                    this));

        return bottomSysId.quasistatic(Direction.kForward)
            .andThen(new WaitCommand(2))
            .andThen(Commands.runOnce(() -> this.BottomLaunchMotor.getEncoder().setPosition(0)))
            .andThen(bottomSysId.quasistatic(Direction.kReverse))
            .andThen(new WaitCommand(2))
            .andThen(Commands.runOnce(() -> this.BottomLaunchMotor.getEncoder().setPosition(0)))
            .andThen(bottomSysId.dynamic(Direction.kForward))
            .andThen(new WaitCommand(2))
            .andThen(Commands.runOnce(() -> this.BottomLaunchMotor.getEncoder().setPosition(0)))
            .andThen(bottomSysId.dynamic(Direction.kReverse))
            .finallyDo(() -> this.Stop());
    }
}
