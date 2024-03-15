package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.RobotSubsystem;
import frc.robot.Constants;
import frc.robot.Controllers;
import frc.robot.FieldConstants;
import frc.robot.Robot;

public class DriveSubsystem extends RobotSubsystem<Robot>
{
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/introduction.html
    public DifferentialDrivetrainSim SimulatedDrive;

    // https://www.revrobotics.com/rev-21-1650/
    private CANSparkMax LeftLeadMotor, RightLeadMotor, LeftFollowMotor, RightFollowMotor;
    public DifferentialDriveKinematics Kinematics = new DifferentialDriveKinematics(Constants.Drivetrain.TrackWidth);
    
    private SimpleMotorFeedforward FeedForward = new SimpleMotorFeedforward(0.10158, 2, 0.53799);

    private ChassisSpeeds Speeds = new ChassisSpeeds();

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

        this.LeftLeadMotor.setInverted(true);
        this.RightLeadMotor.setInverted(false);

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


        var startingPosition = FieldConstants.GetStartingPosition();
        if (startingPosition.isPresent())
        {
            this.SimulatedDrive.setPose(startingPosition.get());
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
            this.RightLeadMotor.setVoltage(FeedForward.calculate(setWheelSpeeds.rightMetersPerSecond, 0.05));
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
}