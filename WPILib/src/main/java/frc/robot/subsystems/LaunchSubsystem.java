package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.I2C.Port;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.ColorSensorV3.ProximitySensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ProximitySensorResolution;

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
import frc.robot.commands.RampCommand;
import frc.robot.math.SpeakerCalculations;

public class LaunchSubsystem extends RobotSubsystem<Robot>
{
    /**
     * Any value larger than this threshold from the sensor will mark a detected note. 
     */
    public static final int NoteProximityThreshold = 150;

    // https://www.revrobotics.com/rev-21-1650/
    public CANSparkMax IntakeMotor, FeederMotor, TopLaunchMotor, BottomLaunchMotor;

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
        this.BottomLaunchMotor = new CANSparkMax(7, MotorType.kBrushless);

        this.TopLaunchMotor.getEncoder().setPositionConversionFactor(Constants.Launcher.WheelCircumference);
        this.TopLaunchMotor.getEncoder().setVelocityConversionFactor(Constants.Launcher.WheelCircumference);
        this.BottomLaunchMotor.getEncoder().setPositionConversionFactor(Constants.Launcher.WheelCircumference);
        this.BottomLaunchMotor.getEncoder().setVelocityConversionFactor(Constants.Launcher.WheelCircumference);

        this.IntakeMotor = new CANSparkMax(Constants.GroundIntake.CANPort, MotorType.kBrushless);
        this.FeederMotor = new CANSparkMax(9, MotorType.kBrushless);

        this.TopLaunchMotor.setInverted(false);
        this.BottomLaunchMotor.setInverted(false);
        this.IntakeMotor.setInverted(true);
        this.FeederMotor.setInverted(true);

        // May hurt the shooter motors lol
        this.TopLaunchMotor.setIdleMode(IdleMode.kBrake);
        this.BottomLaunchMotor.setIdleMode(IdleMode.kBrake);
        this.FeederMotor.setIdleMode(IdleMode.kBrake);
        this.IntakeMotor.setIdleMode(IdleMode.kCoast);

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
            this.IntakeMotor.stopMotor();
            this.FeederMotor.stopMotor();
            this.TopLaunchMotor.stopMotor();
            this.BottomLaunchMotor.stopMotor();
        }
    }

    public Command Feed(double speed)
    {
        return this.runEnd(() -> this.FeederMotor.set(speed), () -> this.FeederMotor.stopMotor());
    }

    public Command AutoIntake()
    {
        if (RobotBase.isSimulation()) return Commands.waitSeconds(2);
        
        return this.runEnd(() -> { this.FeederMotor.set(0.4); this.IntakeMotor.set(0.8); }, () -> { this.FeederMotor.stopMotor(); this.IntakeMotor.stopMotor(); })
            .until(() -> this.NoteSensor.getProximity() > 200)
            .withName("Intake");
    }

    public Command Launch(double percentageTop, double percentageBottom)
    {
        if (RobotBase.isSimulation()) return Commands.waitSeconds(2);

        return new RampCommand(0, percentageTop, 1 / 2, (speed) -> this.SetLaunchSpeed(speed), this)
            .andThen(this.Feed(0.3).withTimeout(1))
            .finallyDo(() -> this.Stop())
            .withName("Launch Note");
    }
    public Command AutoLaunch()
    {
        if (RobotBase.isSimulation()) return Commands.waitSeconds(2);

        double percentage = SpeakerCalculations.CalculateLaunchPercentageForSpeaker();

        System.out.println("Shooting at " + percentage + " percent!");

        return new RampCommand(this.TopLaunchMotor.get(), percentage, 1, (speed) -> this.SetLaunchSpeed(speed), this)
            .andThen(this.Feed(0.3).withTimeout(0.5))
            .finallyDo(() -> this.Stop())
            .withName("Launch Note");
    }

    @Override
    public void initSendable(SendableBuilder builder)
    {
        // builder.addDoubleProperty("Top Launcher Speed", () -> this.GetSpeeds().TopMetersPerSecond, null);
        // builder.addDoubleProperty("Bottom Launcher Speed", () -> this.GetSpeeds().BottomMetersPerSecond, null);
        builder.addBooleanProperty("Note Loaded", () -> this.IsLoaded(), null);
        // builder.addBooleanProperty("Note Sensor Connected", () -> this.NoteSensor.isConnected(), null);

        if (RobotBase.isReal())
        {
            builder.addDoubleProperty("Note Proximity", () -> this.NoteSensor.getProximity(), null);
        }
    }
}