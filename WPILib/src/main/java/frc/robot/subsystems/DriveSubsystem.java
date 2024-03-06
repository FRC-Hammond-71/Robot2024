package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.Optional;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.DifferentialDriveAccelerationLimiter;
import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.RobotSubsystem;
import frc.robot.Constants;
import frc.robot.Controllers;
import frc.robot.FieldGeometry;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.GameCommands;
import frc.robot.commands.PathCommands;

public class DriveSubsystem extends RobotSubsystem<Robot>
{
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/introduction.html

    public DifferentialDrivetrainSim SimulatedDrive;

    // https://www.revrobotics.com/rev-21-1650/
    private CANSparkMax LeftLeadMotor, RightLeadMotor, LeftFollowMotor, RightFollowMotor;
    public DifferentialDriveKinematics Kinematics = new DifferentialDriveKinematics(Constants.Drivetrain.TrackWidth);
    private SimpleMotorFeedforward FeedForward = new SimpleMotorFeedforward(0.10158, 2, 0.53799);

    // private SimpleMotorFeedforward FeedForward = new
    // SimpleMotorFeedforward(0.10158, 2.161, 0.53799);

    // private PIDController LeftMotorsPID = new PIDController(.05, 0, 0);
    // private PIDController RightMotorsPID = new PIDController(.05,0,0);

    private ChassisSpeeds Speeds = new ChassisSpeeds();
    private Optional<DifferentialDriveWheelVoltages> OverrideVoltages = Optional.empty();

    public DriveSubsystem(Robot robot)
    {
        super(robot);
    }

    @Override
    protected void initializeReal()
    {
        this.LeftLeadMotor = new CANSparkMax(1, MotorType.kBrushless);
        this.LeftFollowMotor = new CANSparkMax(2, MotorType.kBrushless);
        this.RightFollowMotor = new CANSparkMax(3, MotorType.kBrushless);
        this.RightLeadMotor = new CANSparkMax(4, MotorType.kBrushless);

        this.LeftLeadMotor.setInverted(false);
        this.RightLeadMotor.setInverted(true);

        // this.RightFollowMotor.setIdleMode(IdleMode.kBrake);
        // this.RightLeadMotor.setIdleMode(IdleMode.kCoast);
        // this.LeftFollowMotor.setIdleMode(IdleMode.kBrake);
        // this.LeftLeadMotor.setIdleMode(IdleMode.kCoast);
        this.SetIdle(IdleMode.kBrake);

        this.LeftFollowMotor.follow(this.LeftLeadMotor);
        this.RightFollowMotor.follow(this.RightLeadMotor);

        this.LeftLeadMotor.getEncoder().setPosition(0);
        this.RightLeadMotor.getEncoder().setPosition(0);
    }

    @Override
    protected void initializeSimulated()
    {
        this.SimulatedDrive = new DifferentialDrivetrainSim(
            DCMotor.getNEO(2),
            Constants.Drivetrain.WheelGearing,
            8,
            edu.wpi.first.math.util.Units.lbsToKilograms(Constants.RobotWeight),
            Constants.Drivetrain.WheelRadius,
            Constants.Drivetrain.TrackWidth,
            null
        // VecBuilder.fill(0.001, 0.001, 0.001, 0.05, 0.05, 0.005, 0.005)
        );
        try 
        {
            this.SimulatedDrive.setPose(FieldGeometry.GetStartingPosition());
        }
        catch (Exception ex)
        {
            System.out.println("Could not set inital position in Simulation!");
        }
    }

    public DifferentialDriveWheelPositions GetWheelPositions()
    {
        return new DifferentialDriveWheelPositions(
            RobotBase.isReal() 
                ? this.GetLeftWheelPosition()
                : this.SimulatedDrive.getLeftPositionMeters(),
            RobotBase.isReal() 
                ? this.GetRightWheelPosition()
                : this.SimulatedDrive.getRightPositionMeters());
    }

    /**
     * @return Left wheel velocity in M/s
     */
    public double GetLeftWheelVelocity()
    {
        return (this.LeftLeadMotor.getEncoder().getVelocity() / 60) / Constants.Drivetrain.WheelGearing * 2 * Math.PI * Constants.Drivetrain.WheelRadius;
    }

    /**
     * @return Right wheel velocity in M/s
     */
    public double GetRightWheelVelocity()
    {
        return (this.RightLeadMotor.getEncoder().getVelocity() / 60) / Constants.Drivetrain.WheelGearing * 2 * Math.PI * Constants.Drivetrain.WheelRadius;
    }

    /**
     * @return Left wheel velocity in M/s
     */
    public double GetLeftWheelPosition()
    {
        return this.LeftLeadMotor.getEncoder().getPosition() / Constants.Drivetrain.WheelGearing * 2 * Math.PI * Constants.Drivetrain.WheelRadius;
    }

    /**
     * @return Right wheel velocity in M/s
     */
    public double GetRightWheelPosition()
    {
        return this.RightLeadMotor.getEncoder().getPosition() / Constants.Drivetrain.WheelGearing * 2 * Math.PI * Constants.Drivetrain.WheelRadius;
    }

    public void ResetEncoders()
    {
        if (RobotBase.isReal())
        {
            this.LeftLeadMotor.getEncoder().setPosition(0);
            this.RightLeadMotor.getEncoder().setPosition(0);
        } 
        else
        {
            this.SimulatedDrive.setPose(new Pose2d());
        }
    }

    /**
     * Stops all motors and resets the target speed to zero.
     */
    public void Stop()
    {
        // Bypass Set()
        this.Speeds = new ChassisSpeeds(0, 0, 0);
        this.OverrideVoltages = Optional.empty();

        if (RobotBase.isReal())
        {
            this.LeftLeadMotor.stopMotor();
            this.RightLeadMotor.stopMotor();
            // Just in case, stop following motors.
            this.LeftFollowMotor.stopMotor();
            this.RightFollowMotor.stopMotor();
        } 
        else
        {
            this.SimulatedDrive.setInputs(0, 0);
        }
    }

    public ChassisSpeeds GetWheelSpeeds()
    {
        return this.Kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(
            RobotBase.isReal() ? this.GetLeftWheelVelocity() : this.SimulatedDrive.getLeftVelocityMetersPerSecond(),
            RobotBase.isReal() ? this.GetRightWheelVelocity() : this.SimulatedDrive.getRightVelocityMetersPerSecond()));
    }

    public ChassisSpeeds GetSpeeds() 
    {
        return this.Speeds;
    }

    /**
     * Sets the Drivetrain to move at the specified rate.
     * 
     * @return The clamped or rate-limited speeds.
     */
    public void Set(ChassisSpeeds speeds)
    {
        this.Speeds = speeds;
        this.UpdateMotors();
    }
    public void Set(Optional<DifferentialDriveWheelVoltages> voltages)
    {
        this.OverrideVoltages = voltages;
        this.UpdateMotors();
    }
    public void SetArcade(double xSpeed, double zRotation)
    {
        xSpeed = Controllers.ApplyDeadzone(xSpeed);
        xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);

        zRotation = Controllers.ApplyDeadzone(zRotation);
        zRotation = Math.copySign(zRotation * zRotation, zRotation);

        this.Set(new ChassisSpeeds(
            xSpeed * Constants.Drivetrain.MaxXSpeed,
            0,
            Constants.Drivetrain.MaxAngularSpeed.times(zRotation).getRadians()
        ));
    }

    public void SetIdle(IdleMode mode)
    {
        if (RobotBase.isReal())
        {
            this.RightFollowMotor.setIdleMode(mode);
            this.RightLeadMotor.setIdleMode(mode);
            this.LeftFollowMotor.setIdleMode(mode);
            this.LeftLeadMotor.setIdleMode(mode);
        }
    }

    protected void UpdateMotors()
    {
        if (DriverStation.isDisabled())
        {
            this.Stop();
            return;
        }

        var setWheelSpeeds = this.Kinematics.toWheelSpeeds(new ChassisSpeeds(
            Math.min(Math.max(this.Speeds.vxMetersPerSecond, -Constants.Drivetrain.MaxXSpeed), Constants.Drivetrain.MaxXSpeed),
            0,
            Math.min(Math.max(this.Speeds.omegaRadiansPerSecond, -Constants.Drivetrain.MaxAngularSpeed.getRadians()), Constants.Drivetrain.MaxAngularSpeed.getRadians())));

        if (RobotBase.isReal())
        {
            this.LeftLeadMotor.setVoltage(FeedForward.calculate(setWheelSpeeds.leftMetersPerSecond, 0.05));
            this.RightLeadMotor.setVoltage((FeedForward.calculate(setWheelSpeeds.rightMetersPerSecond, 0.05)));
        } 
        else
        {
            this.SimulatedDrive.setInputs(
                FeedForward.calculate(this.SimulatedDrive.getLeftVelocityMetersPerSecond(), setWheelSpeeds.leftMetersPerSecond, 0.02),
                FeedForward.calculate(this.SimulatedDrive.getRightVelocityMetersPerSecond(), setWheelSpeeds.rightMetersPerSecond, 0.02));
        }
    }

    @Override
    public void periodic()
    {
        if (!DriverStation.isDisabled())
        {
            this.UpdateMotors();
        }
    }

    @Override
    public void simulationPeriodic()
    {
        this.SimulatedDrive.update(0.02);
    }

    @Override
    public void initSendable(SendableBuilder builder)
    {
        builder.addDoubleProperty("Desired Speed", () -> this.Speeds.vxMetersPerSecond, null);
        builder.addDoubleProperty("Desired Rotation", () -> edu.wpi.first.math.util.Units.radiansToDegrees(this.Speeds.omegaRadiansPerSecond),
                null);

        builder.addDoubleProperty("Actual Speed", () ->this.GetWheelSpeeds().vxMetersPerSecond, null);
        builder.addDoubleProperty("Actual Rotation",
                () -> edu.wpi.first.math.util.Units.radiansToDegrees(this.GetWheelSpeeds().omegaRadiansPerSecond), null);

        builder.addStringProperty("Blocking Command", () ->
        {
            var requiredCommand = CommandScheduler.getInstance().requiring(this);
            boolean isDefault = requiredCommand == this.getDefaultCommand();

            return requiredCommand != null && !isDefault ? requiredCommand.getName() : "None";
        }, null);
    }

    public Command PerformSysID()
    {
        var sysId = new SysIdRoutine(new SysIdRoutine.Config(

			Units.Volts.of(0.25).per(Units.Seconds.of(1)),
			Units.Volts.of(0.5),
			Units.Seconds.of(3.4)

		), new SysIdRoutine.Mechanism(
			(voltage) -> 
			{
				System.out.println(voltage);

				// Apply voltages to motors.
				this.Set(Optional.of(new DifferentialDriveWheelVoltages(-voltage.magnitude(), -voltage.magnitude())));
			},
			(log) ->
			{
				log.motor("flywheel")
					.voltage(Units.Volts.of(this.LeftLeadMotor.getBusVoltage()))
					.linearVelocity(Units.MetersPerSecond.of(this.LeftLeadMotor.getEncoder().getVelocity() / 60))
					.linearPosition(Units.Meters.of(this.LeftLeadMotor.getEncoder().getPosition()));
			},
			this));

		return sysId
			.quasistatic(Direction.kForward)
			.andThen(Commands.runOnce(() -> System.out.println("Going Back!")))
			.andThen(sysId.quasistatic(Direction.kReverse))
			.andThen(Commands.runOnce(() -> System.out.println("Beginning dynamic test...")))
			.andThen(sysId.dynamic(Direction.kForward))
			.andThen(Commands.runOnce(() -> System.out.println("Going Back!")))
			.andThen(sysId.dynamic(Direction.kReverse))
			.finallyDo(() -> this.Stop());
    }
}
