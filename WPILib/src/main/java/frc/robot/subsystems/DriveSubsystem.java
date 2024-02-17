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
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Controllers;
import frc.robot.RobotContainer;
import frc.robot.commands.GameCommands;
import frc.robot.commands.PathCommands;

public class DriveSubsystem extends SubsystemBase 
{
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/introduction.html

    // ----------
    // Simulation
    // ----------
    public DifferentialDrivetrainSim SimulatedDrive;

    // https://www.revrobotics.com/rev-21-1650/
	public CANSparkMax LeftLeadMotor, RightLeadMotor, LeftFollowMotor, RightFollowMotor;

    public DifferentialDriveKinematics Kinematics = new DifferentialDriveKinematics(Constants.Drivetrain.TrackWidth);

    private SimpleMotorFeedforward FeedForward = new SimpleMotorFeedforward(0.10158, 0.4, 0.53799);

    private MedianFilter InputFilter = new MedianFilter(5);

    // private TrapezoidProfile VelocityProfile = new TrapezoidProfile(new Constraints(Constants.Drivetrain.MaxForwardSpeed, 5));

    // private SimpleMotorFeedforward FeedForward = new SimpleMotorFeedforward(0.10158, 2.161, 0.53799);

    // private SlewRateLimiter ForwardRateLimiter = new SlewRateLimiter(Constants.Drivetrain.MaxForwardSpeedDelta);
    // private SlewRateLimiter RotationRateLimiter = new SlewRateLimiter(Constants.Drivetrain.MaxRotationalSpeedDelta);

    // private PIDController LeftMotorsPID = new PIDController(.05, 0, 0);
    // private PIDController RightMotorsPID = new PIDController(.05,0,0);

    private ChassisSpeeds Speeds = new ChassisSpeeds();

    public Optional<DifferentialDriveWheelVoltages> OverrideVoltages =  Optional.empty();

    public DriveSubsystem()
    {
        super();
        
        if (RobotBase.isReal())
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
            this.LeftLeadMotor.getEncoder().setPositionConversionFactor(Constants.Drivetrain.WheelCircumference / Constants.Drivetrain.WheelGearing);
            this.RightLeadMotor.getEncoder().setPositionConversionFactor(Constants.Drivetrain.WheelCircumference / Constants.Drivetrain.WheelGearing);
            this.LeftLeadMotor.getEncoder().setVelocityConversionFactor(Constants.Drivetrain.WheelCircumference / Constants.Drivetrain.WheelGearing);
            this.RightLeadMotor.getEncoder().setVelocityConversionFactor(Constants.Drivetrain.WheelCircumference / Constants.Drivetrain.WheelGearing);
        }
        else
        {
            // this.SimulatedLeftEncoder = new EncoderSim(this.Left)
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

        setDefaultCommand(Commands.run(() -> 
        {
            if (Controllers.DriverController.getAButtonPressed())
            {
                // GameCommands.AutoRotateAndLaunch().schedule();
            }

            double forward = Controllers.ApplyDeadzone(Controllers.DriverController.getLeftY());
            forward = Math.copySign(Math.pow(forward, 2), forward);

            double rotation = Controllers.ApplyDeadzone(Controllers.DriverController.getRightX());

            this.Set(new ChassisSpeeds
            (
                forward * Constants.Drivetrain.MaxForwardSpeed,
                0,
                Rotation2d.fromDegrees(rotation * 360).getRadians()
            ));
        }, this));
    }

    public DifferentialDriveWheelPositions GetWheelPositions()
    {
        return new DifferentialDriveWheelPositions(
            RobotBase.isReal() ? this.LeftLeadMotor.getEncoder().getPosition() : this.SimulatedDrive.getLeftPositionMeters(),
            RobotBase.isReal() ? this.RightLeadMotor.getEncoder().getPosition() : this.SimulatedDrive.getRightPositionMeters());
    }

    public void ResetEncoders()
    {
        if (RobotBase.isReal())
        {
            this.LeftLeadMotor.getEncoder().setPosition(0);
            this.RightLeadMotor.getEncoder().setPosition(0);
        }
        else this.SimulatedDrive.setPose(new Pose2d());
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
            RobotBase.isReal() ? this.LeftLeadMotor.getEncoder().getVelocity() / 60 : this.SimulatedDrive.getLeftVelocityMetersPerSecond(),
            RobotBase.isReal() ? this.RightLeadMotor.getEncoder().getVelocity() / 60 : this.SimulatedDrive.getRightVelocityMetersPerSecond()
        ));
    }

    public ChassisSpeeds GetSpeeds() { return this.Speeds; }

    /**
     * Sets the Drivetrain to move at the specified rate.
     * @return The clamped or rate-limited speeds.
     */
    public void Set(ChassisSpeeds speeds)
    {
        this.Speeds = speeds;
        this.UpdateMotors();
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
            this.Speeds.omegaRadiansPerSecond
        ));

        if (RobotBase.isReal())
        {
            this.LeftLeadMotor.setVoltage(FeedForward.calculate(nextWheelSpeeds.leftMetersPerSecond, 0.25));
            this.RightLeadMotor.setVoltage(FeedForward.calculate(nextWheelSpeeds.rightMetersPerSecond, 0.25));
        }
        else
        {
            this.SimulatedDrive.setInputs(
                InputFilter.calculate(FeedForward.calculate(this.SimulatedDrive.getLeftVelocityMetersPerSecond(), nextWheelSpeeds.leftMetersPerSecond, 0.02)),
                InputFilter.calculate(FeedForward.calculate(this.SimulatedDrive.getRightVelocityMetersPerSecond(), nextWheelSpeeds.rightMetersPerSecond, 0.02)));
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
            // builder.addDoubleProperty("Left Encoder Position", () -> this.LeftLeadMotor.getEncoder().getPosition(), null);
            // builder.addDoubleProperty("Right Encoder Position", () -> this.LeftLeadMotor.getEncoder().getPosition(), null);
        }

        builder.addDoubleProperty("Desired Speed", () -> this.Speeds.vxMetersPerSecond, null);
        builder.addDoubleProperty("Desired Rotation", () -> Units.radiansToDegrees(this.Speeds.omegaRadiansPerSecond), null);

        builder.addDoubleProperty("Actual Speed", () -> this.GetWheelSpeeds().vxMetersPerSecond, null);
        builder.addDoubleProperty("Actual Rotation", () -> Units.radiansToDegrees(this.GetWheelSpeeds().omegaRadiansPerSecond), null);

        // builder.addDoubleProperty("FEED Ks", () -> this.FeedForward.ks, (v) -> this.FeedForward = new SimpleMotorFeedforward(v, this.FeedForward.kv, this.FeedForward.ka));
        // builder.addDoubleProperty("FEED Kv", () -> this.FeedForward.kv, (v) -> this.FeedForward = new SimpleMotorFeedforward(this.FeedForward.ks, v, this.FeedForward.ka));
        // builder.addDoubleProperty("FEED Ka", () -> this.FeedForward.ks, (v) -> this.FeedForward = new SimpleMotorFeedforward(this.FeedForward.ks, this.FeedForward.kv, v));
    
        builder.addStringProperty("Blocking Command", () -> {

            var requiredCommand = CommandScheduler.getInstance().requiring(this);
            boolean isDefault = requiredCommand == this.getDefaultCommand();
           
            return requiredCommand != null && !isDefault ? requiredCommand.getName() :  "None";

        }, null);
    }
}
