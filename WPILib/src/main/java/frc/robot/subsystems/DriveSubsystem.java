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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.RobotSubsystem;
import frc.robot.Constants;
import frc.robot.Controllers;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.GameCommands;
import frc.robot.commands.PathCommands;

public class DriveSubsystem extends RobotSubsystem<Robot>
{
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/introduction.html

    public DifferentialDrivetrainSim SimulatedDrive;

    // https://www.revrobotics.com/rev-21-1650/
    public CANSparkMax LeftLeadMotor, RightLeadMotor, LeftFollowMotor, RightFollowMotor;

    public DifferentialDriveKinematics Kinematics = new DifferentialDriveKinematics(Constants.Drivetrain.TrackWidth);

    private SimpleMotorFeedforward FeedForward = new SimpleMotorFeedforward(0.10158, 2, 0.53799);

    private MedianFilter InputFilter = new MedianFilter(5);

    // private TrapezoidProfile VelocityProfile = new TrapezoidProfile(new
    // Constraints(Constants.Drivetrain.MaxForwardSpeed, 5));

    // private SimpleMotorFeedforward FeedForward = new
    // SimpleMotorFeedforward(0.10158, 2.161, 0.53799);

    // private SlewRateLimiter ForwardRateLimiter = new
    // SlewRateLimiter(Constants.Drivetrain.MaxForwardSpeedDelta);
    // private SlewRateLimiter RotationRateLimiter = new
    // SlewRateLimiter(Constants.Drivetrain.MaxRotationalSpeedDelta);

    // private PIDController LeftMotorsPID = new PIDController(.05, 0, 0);
    // private PIDController RightMotorsPID = new PIDController(.05,0,0);

    private ChassisSpeeds Speeds = new ChassisSpeeds();

    public Optional<DifferentialDriveWheelVoltages> OverrideVoltages = Optional.empty();

    public DriveSubsystem(Robot robot)
    {
        super(robot);
    }

    @Override
    protected void initializeReal()
    {
        this.LeftLeadMotor = new CANSparkMax(1, MotorType.kBrushless);
        this.RightLeadMotor = new CANSparkMax(4, MotorType.kBrushless);
        this.LeftFollowMotor = new CANSparkMax(2, MotorType.kBrushless);
        this.RightFollowMotor = new CANSparkMax(3, MotorType.kBrushless);

        this.LeftLeadMotor.setInverted(true);
        this.RightLeadMotor.setInverted(false);

        this.RightFollowMotor.setIdleMode(IdleMode.kCoast);
        this.RightLeadMotor.setIdleMode(IdleMode.kBrake);
        this.LeftFollowMotor.setIdleMode(IdleMode.kCoast);
        this.LeftLeadMotor.setIdleMode(IdleMode.kBrake);

        this.LeftFollowMotor.follow(this.LeftLeadMotor);
        this.RightFollowMotor.follow(this.RightLeadMotor);

        this.LeftLeadMotor.getEncoder().setPosition(0);
        this.RightLeadMotor.getEncoder().setPosition(0);
        this.LeftLeadMotor.getEncoder().setPositionConversionFactor(
            -Constants.Drivetrain.WheelCircumference / Constants.Drivetrain.WheelGearing);
        this.RightLeadMotor.getEncoder().setPositionConversionFactor(
            -Constants.Drivetrain.WheelCircumference / Constants.Drivetrain.WheelGearing);
        this.LeftLeadMotor.getEncoder().setVelocityConversionFactor(
            -Constants.Drivetrain.WheelCircumference / Constants.Drivetrain.WheelGearing);
        this.RightLeadMotor.getEncoder().setVelocityConversionFactor(
            -Constants.Drivetrain.WheelCircumference / Constants.Drivetrain.WheelGearing);
    }

    @Override
    protected void initializeSimulated()
    {
        this.SimulatedDrive = new DifferentialDrivetrainSim(
            DCMotor.getNEO(2),
            Constants.Drivetrain.WheelGearing,
            8,
            Units.lbsToKilograms(Constants.RobotWeight),
            Constants.Drivetrain.WheelRadius,
            Constants.Drivetrain.TrackWidth,
            null
        // VecBuilder.fill(0.001, 0.001, 0.001, 0.05, 0.05, 0.005, 0.005)
        );
    }

    public DifferentialDriveWheelPositions GetWheelPositions()
    {
        return new DifferentialDriveWheelPositions(
            RobotBase.isReal() 
                ? this.LeftLeadMotor.getEncoder().getPosition()
                : this.SimulatedDrive.getLeftPositionMeters(),
            RobotBase.isReal() 
                ? this.RightLeadMotor.getEncoder().getPosition()
                : this.SimulatedDrive.getRightPositionMeters());
    }

    /**
     * @return Left wheel velocity in M/s
     */
    public double GetLeftWheelVelocity()
    {
        return (this.LeftLeadMotor.getEncoder().getVelocity() + this.LeftFollowMotor.getEncoder().getVelocity()) / 2
                / 60;
    }

    /**
     * @return Right wheel velocity in M/s
     */
    public double GetRightWheelVelocity()
    {
        return (this.RightLeadMotor.getEncoder().getVelocity() + this.RightFollowMotor.getEncoder().getVelocity()) / 2
                / 60;
    }

    /**
     * @return Left wheel velocity in M/s
     */
    public double GetLeftWheelPosition()
    {
        return (this.LeftLeadMotor.getEncoder().getPosition() + this.LeftFollowMotor.getEncoder().getPosition()) / 2;
    }

    /**
     * @return Right wheel velocity in M/s
     */
    public double GetRightWheelPosition()
    {
        return (this.RightLeadMotor.getEncoder().getPosition() + this.RightFollowMotor.getEncoder().getPosition()) / 2;
    }

    public void ResetEncoders()
    {
        if (RobotBase.isReal())
        {
            this.LeftLeadMotor.getEncoder().setPosition(0);
            this.RightLeadMotor.getEncoder().setPosition(0);
        } else
            this.SimulatedDrive.setPose(new Pose2d());
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
            RobotBase.isReal() ? this.GetRightWheelVelocity()
                : this.SimulatedDrive.getRightVelocityMetersPerSecond()));
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
        xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
        xSpeed = Controllers.ApplyDeadzone(xSpeed);

        zRotation = Math.copySign(zRotation * zRotation, zRotation);
        zRotation = Controllers.ApplyDeadzone(zRotation);

        this.Set(new ChassisSpeeds(
            xSpeed * Constants.Drivetrain.MaxForwardSpeed,
            0,
            Constants.Drivetrain.MaxRotationalSpeed.times(zRotation).getRadians()
        ));
    }

    protected void UpdateMotors()
    {
        if (this.OverrideVoltages.isPresent())
        {
            this.LeftLeadMotor.set(this.OverrideVoltages.get().left);
            this.RightLeadMotor.set(this.OverrideVoltages.get().right);
            return;
        }

        // Apply rate-limits
        var nextWheelSpeeds = this.Kinematics.toWheelSpeeds(new ChassisSpeeds(
                this.Speeds.vxMetersPerSecond,
                0,
                this.Speeds.omegaRadiansPerSecond));

        if (RobotBase.isReal())
        {
            this.LeftLeadMotor.setVoltage(FeedForward.calculate(nextWheelSpeeds.leftMetersPerSecond, 0.15));
            this.RightLeadMotor.setVoltage(FeedForward.calculate(nextWheelSpeeds.rightMetersPerSecond, 0.15));
        } 
        else
        {
            this.SimulatedDrive.setInputs(
                InputFilter.calculate(
                    FeedForward.calculate(this.SimulatedDrive.getLeftVelocityMetersPerSecond(),
                    nextWheelSpeeds.leftMetersPerSecond, 0.02)),

                InputFilter.calculate(FeedForward.calculate(
                    this.SimulatedDrive.getRightVelocityMetersPerSecond(),
                    nextWheelSpeeds.rightMetersPerSecond, 0.02)));
        }
    }

    @Override
    public void periodic()
    {
        this.UpdateMotors();
    }

    @Override
    public void simulationPeriodic()
    {
        this.SimulatedDrive.update(0.02);
    }

    @Override
    public void initSendable(SendableBuilder builder)
    {
        if (RobotBase.isReal())
        {
            builder.addDoubleProperty("Left Encoder Velocity", this::GetLeftWheelVelocity, null);
            builder.addDoubleProperty("Right Encoder Velocity", this::GetRightWheelVelocity, null);
        }

        builder.addDoubleProperty("Desired Speed", () -> this.Speeds.vxMetersPerSecond, null);
        builder.addDoubleProperty("Desired Rotation", () -> Units.radiansToDegrees(this.Speeds.omegaRadiansPerSecond),
                null);

        builder.addDoubleProperty("Actual Speed", () ->this.GetWheelSpeeds().vxMetersPerSecond, null);
        builder.addDoubleProperty("Actual Rotation",
                () -> Units.radiansToDegrees(this.GetWheelSpeeds().omegaRadiansPerSecond), null);

        builder.addStringProperty("Blocking Command", () ->
        {
            var requiredCommand = CommandScheduler.getInstance().requiring(this);
            boolean isDefault = requiredCommand == this.getDefaultCommand();

            return requiredCommand != null && !isDefault ? requiredCommand.getName() : "None";
        }, null);
    }
}
