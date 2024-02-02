package frc.robot.subsystems;

import java.util.Date;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Controllers;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;

public class DriveSubsystem extends SubsystemBase 
{
    public DifferentialDriveKinematics Kinematics = new DifferentialDriveKinematics(Constants.Drivetrain.TrackWidth);

    private SimpleMotorFeedforward FeedForward = new SimpleMotorFeedforward(0.10158, 2.161, 0.53799);
    // ks = the static voltage required to initially move wheels
    // kv = The voltage to keep moving at a velocity
    // ka = Voltage required to accelerate at the desired acceleration

    // More reduction in gear-ratio provides more torque - less faster speeds.

    // Kt = Motor torque constant
    // Kv rating provides an estimate of how many rotations a motor will undergo for every volt applied to it.

    // Kv = Free Spinning RPM (Max Motor Speed with no Load)
    // ω is the rotational speed of the motor

    // Stall Torque = 2.6 Nm
    // Stall Current (I) = 1.8 A

    // ω = Kv * V
    // ω = 5676 RPM * 12
    // τ = Kt * (V/R) 
    // Stall Torque 

    // ------
	// Motors
	// ------
	private CANSparkMax LeftLeadMotor;
	private CANSparkMax RightLeadMotor;
	private CANSparkMax LeftFollowMotor;
	private CANSparkMax RightFollowMotor;
    // ------
    // Motor PID (do we want this?)
    // ------
    private PIDController LeftMotorsPID = new PIDController(.05, 0, 0);
    private PIDController RightMotorsPID = new PIDController(.05,0,0);
    // -------
    // Sensors
    // -------
    public AHRS IMU = new AHRS(SPI.Port.kMXP);

    // https://www.chiefdelphi.com/t/considering-the-intricacies-of-autonomous-pathfinding-algorithms-and-the-myriad-of-sensor-fusion-techniques-in-frc-how-might-one-harmonize-the-celestial-dance-of-encoder-ticks-and-gyroscopic-precession/441692/28
    public DriveSubsystem()
    {
        super();
        
        if (RobotBase.isReal())
        {
            this.LeftLeadMotor = new CANSparkMax(5, MotorType.kBrushless);
            this.RightLeadMotor =  new CANSparkMax(4, MotorType.kBrushless);
            this.LeftFollowMotor = new CANSparkMax(3, MotorType.kBrushless);
            this.RightFollowMotor = new CANSparkMax(2, MotorType.kBrushless);

            this.LeftLeadMotor.setInverted(true);
            this.RightLeadMotor.setInverted(false);
            
            this.LeftFollowMotor.follow(this.LeftLeadMotor);
            this.RightFollowMotor.follow(this.RightLeadMotor);
            
            this.LeftLeadMotor.getEncoder().setPosition(0);
            this.RightLeadMotor.getEncoder().setPosition(0);
            this.LeftLeadMotor.getEncoder().setPositionConversionFactor(Constants.Drivetrain.WheelCircumference / Constants.Drivetrain.WheelGearing);
            this.RightLeadMotor.getEncoder().setPositionConversionFactor(Constants.Drivetrain.WheelCircumference / Constants.Drivetrain.WheelGearing);
            this.LeftLeadMotor.getEncoder().setVelocityConversionFactor(Constants.Drivetrain.WheelCircumference / Constants.Drivetrain.WheelGearing);
            this.RightLeadMotor.getEncoder().setVelocityConversionFactor(Constants.Drivetrain.WheelCircumference / Constants.Drivetrain.WheelGearing);
            
            System.out.println(String.format("Encoder CountsPerRevolution %s", this.RightLeadMotor.getEncoder().getCountsPerRevolution()));
        }
        else
        {
            // TODO: Support simulation of Robot
            throw new UnsupportedOperationException("DriveSubsystem cannot be simulated!");
        }

        setDefaultCommand(Commands.run(() -> {

            double forward = Controllers.DriverController.getLeftY();
            forward = forward > -Constants.Controllers.Deadzone && forward < Constants.Controllers.Deadzone ? 0 : forward;

            double rotation = Controllers.DriverController.getRightX();
            rotation = rotation > -Constants.Controllers.Deadzone && rotation < Constants.Controllers.Deadzone ? 0 : rotation;

            this.Set(new ChassisSpeeds
            (
                forward * Constants.Drivetrain.MaxForwardSpeed,
                0,
                Constants.Drivetrain.MaxRotationalSpeed.times(rotation).getRadians()
            ));
        }));
    }

    public DifferentialDriveWheelPositions GetWheelPositions()
    {
        return new DifferentialDriveWheelPositions(
            this.LeftLeadMotor.getEncoder().getPosition(),
            this.RightLeadMotor.getEncoder().getPosition());
    }

    /**
     * Reset Encoder positions and IMU.
     */
    public void ResetSensors()
    {
        this.LeftLeadMotor.getEncoder().setPosition(0);
		this.RightLeadMotor.getEncoder().setPosition(0);
		this.IMU.reset();
    }

    public void Set(ChassisSpeeds speeds)
    {
        var nextWheelSpeeds = this.Kinematics.toWheelSpeeds(speeds); 

        this.RightLeadMotor.setVoltage(FeedForward.calculate(this.RightLeadMotor.getEncoder().getVelocity() / 60, nextWheelSpeeds.rightMetersPerSecond, 0.02));
        this.LeftLeadMotor.setVoltage(FeedForward.calculate(this.LeftLeadMotor.getEncoder().getVelocity() / 60, nextWheelSpeeds.leftMetersPerSecond, 0.02));
    }

    public void Stop()
    {
        this.LeftLeadMotor.stopMotor();
        this.RightLeadMotor.stopMotor();
    }

    @Override
    public void periodic() 
    {
        // TODO: Add a watchdog?
        
        // Display where the robot is projected to be in the next second
        // this.Field.getObject("Projected Robot").setRobotPose(this.PoseEstimator.getEstimatedPosition());
    }

    @Override
    public void initSendable(SendableBuilder builder) 
    {
        builder.addDoubleProperty("Left Encoder Position", () -> this.LeftLeadMotor.getEncoder().getPosition(), null);
        builder.addDoubleProperty("Right Encoder Position", () -> this.LeftLeadMotor.getEncoder().getPosition(), null);
    }

}


