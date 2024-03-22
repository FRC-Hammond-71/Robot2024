package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.I2C.Port;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.ColorSensorV3.ProximitySensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ProximitySensorResolution;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.RobotSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
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

    private ColorSensorV3 NoteSensor;

    public LaunchSpeeds TargetSpeeds = new LaunchSpeeds();

    private final BooleanLogEntry NoteSensorLog;

    public LaunchSubsystem(Robot robot)
    {
        super(robot);

        this.NoteSensorLog = new BooleanLogEntry(DataLogManager.getLog(), "launcher/noteSensorConnected");
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

    @Override
    public void periodic()
    {
        this.NoteSensorLog.append(this.IsNoteSensorConnected());
    }

    /**
     * @return Whether or not the launcher is loaded with a note.
     */
    public boolean IsLoaded()
    {
        return this.IsNoteSensorConnected() ? this.NoteSensor.getProximity() > NoteProximityThreshold : false;
    }

    public boolean IsNoteSensorConnected()
    {
        return RobotBase.isReal() ? this.NoteSensor.isConnected() : false;
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
        this.SetLaunchSpeed(percentage, percentage);
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
        return this.runEnd(() -> this.FeederMotor.set(speed), () -> this.FeederMotor.stopMotor())
            .withName("Feed");
    }

    /**
     * Feed the loaded note into the Launcher until it is no longer detected!
     */
    public Command AutoFeed()
    {
        return this.Feed(0.3)
            .until(() -> !this.IsLoaded())
            .withName("Auto Feed");
    }

    public Command AutoIntake()
    {
        if (RobotBase.isSimulation()) return Commands.waitSeconds(2);
        
        return this.runEnd(() -> { this.FeederMotor.set(0.4); this.IntakeMotor.set(0.5); }, () -> { this.FeederMotor.stopMotor(); this.IntakeMotor.stopMotor(); })
            .until(() -> this.IsLoaded())
            .withName("Auto Intake");
    }

    public Command Launch(double percentageTop, double percentageBottom)
    {
        if (RobotBase.isSimulation()) return Commands.waitSeconds(2);

        return new RampCommand(0, percentageTop, 1, (speed) -> this.SetLaunchSpeed(speed), this)
            .andThen(this.AutoFeed().withTimeout(1.5))
            .finallyDo((interrupted) -> 
            {
                this.Stop();

                if (!interrupted) 
                {
                    // We just fired a shot, switch to intake mode!
                    Robot.Arm.Mode = ArmPosition.Intake;
                } 
            })
            .withName("Launch Note");
    }

    public Command AutoLaunch()
    {
        if (RobotBase.isSimulation()) return Commands.waitSeconds(2);

        double percentage = SpeakerCalculations.CalculateLaunchPercentageForSpeaker();

        System.out.println("Shooting at " + percentage + " percent!");

        return new RampCommand(0, percentage, 1, (speed) -> this.SetLaunchSpeed(speed), this)
            .andThen(this.AutoFeed().withTimeout(1.5))
            .finallyDo((interrupted) -> 
            {
                this.Stop();

                if (!interrupted) 
                {
                    // We just fired a shot, switch to intake mode!
                    Robot.Arm.Mode = ArmPosition.Intake;
                } 
            })
            .withName("Auto Launch Note");
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