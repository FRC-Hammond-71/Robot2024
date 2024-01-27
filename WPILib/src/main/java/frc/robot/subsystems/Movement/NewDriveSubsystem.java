package frc.robot.subsystems.Movement;

import java.util.Date;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.commands.FaceAtCommand;

public class NewDriveSubsystem extends SubsystemBase 
{
    private DifferentialDriveKinematics Kinematics = new DifferentialDriveKinematics(Constants.Drivetrain.TrackWidth);
    private SimpleMotorFeedforward FeedForward = new SimpleMotorFeedforward(0.10158, 2.161, 0.53799);
    private DifferentialDrivePoseEstimator PoseEstimator;
    
    // ------
	// Motors
	// ------
	private CANSparkMax LeftLeadMotor = new CANSparkMax(5, MotorType.kBrushless);
	private CANSparkMax RightLeadMotor = new CANSparkMax(4, MotorType.kBrushless);
	private CANSparkMax LeftFollowMotor = new CANSparkMax(3, MotorType.kBrushless);
	private CANSparkMax RightFollowMotor = new CANSparkMax(2, MotorType.kBrushless);

    // -------
    // Sensors
    // -------
    private AHRS IMU = new AHRS(SPI.Port.kMXP);

    // -------------
    // Visualization
    // -------------
    private Field2d Field = new Field2d();

    // -----------
    // Interaction
    // -----------
    public XboxController DriverController = new XboxController(Constants.Controllers.DriverPort);

    public ChassisSpeeds Speeds = new ChassisSpeeds();

    // https://www.chiefdelphi.com/t/considering-the-intricacies-of-autonomous-pathfinding-algorithms-and-the-myriad-of-sensor-fusion-techniques-in-frc-how-might-one-harmonize-the-celestial-dance-of-encoder-ticks-and-gyroscopic-precession/441692/28
    public NewDriveSubsystem()
    {
        super();

        SmartDashboard.putData(this.Field);
        
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

        this.PoseEstimator = new DifferentialDrivePoseEstimator(Kinematics, Rotation2d.fromDegrees(0), 0, 0, new Pose2d());

        setDefaultCommand(Commands.run(() -> {

            if (DriverController.getLeftBumperPressed())
            {
                System.out.println("Reset position!");

                this.Reset(new Pose2d(14.6, 5.64, Rotation2d.fromDegrees(0)));
            }

            double forward = this.DriverController.getLeftY();
            forward = forward > -Constants.Controllers.Deadzone && forward < Constants.Controllers.Deadzone ? 0 : forward;

            double rotation = this.DriverController.getRightX();
            rotation = rotation > -Constants.Controllers.Deadzone && rotation < Constants.Controllers.Deadzone ? 0 : rotation;

            this.Speeds = new ChassisSpeeds
            (
                forward * Constants.Drivetrain.MaxForwardSpeed,
                0,
                Constants.Drivetrain.MaxRotationalSpeed.times(rotation).getRadians()
            );
        }, this));
    }

    public void Reset(Pose2d initialPosition)
    {
        this.LeftLeadMotor.getEncoder().setPosition(0);
		this.RightLeadMotor.getEncoder().setPosition(0);
		this.PoseEstimator.resetPosition(initialPosition.getRotation(), 0, 0, initialPosition);
		this.IMU.setAngleAdjustment(initialPosition.getRotation().getDegrees());
		this.IMU.reset();
    }

    public boolean HasVisionPosition()
    {
        return LimelightHelpers.getTV("limelight");
    }
    public VisionPoseMeasurement GetVisionPosition()
    {
        if (!this.HasVisionPosition()) return null;

        var pose = LimelightHelpers.getBotPose2d_wpiBlue("limelight");

        return pose.getX() == 0 && pose.getY() == 0 && pose.getRotation().getDegrees() == 0 ? null : new VisionPoseMeasurement(
            pose,
            Timer.getFPGATimestamp() - LimelightHelpers.getLatency_Capture("limelight") - LimelightHelpers.getLatency_Pipeline("limelight")
        );
    }

    @Override
    public void periodic() 
    {
        this.PoseEstimator.update(Rotation2d.fromDegrees(this.IMU.getAngle()), this.LeftLeadMotor.getEncoder().getPosition(), this.RightLeadMotor.getEncoder().getPosition());

        if (HasVisionPosition())
        {
            var visionMeasurement = this.GetVisionPosition();

            this.PoseEstimator.addVisionMeasurement(visionMeasurement.Pose, visionMeasurement.BeganComputingAt);
        }

        var nextWheelSpeeds = this.Kinematics.toWheelSpeeds(this.Speeds); 

        this.RightLeadMotor.setVoltage(FeedForward.calculate(this.RightLeadMotor.getEncoder().getVelocity() / 60, nextWheelSpeeds.rightMetersPerSecond, 0.02));
        this.LeftLeadMotor.setVoltage(FeedForward.calculate(this.LeftLeadMotor.getEncoder().getVelocity() / 60, nextWheelSpeeds.leftMetersPerSecond, 0.02));
    }

    @Override
    public void initSendable(SendableBuilder builder) 
    {
        builder.addDoubleProperty("Left Encoder Position", () -> this.LeftLeadMotor.getEncoder().getPosition(), null);
        builder.addDoubleProperty("Right Encoder Position", () -> this.LeftLeadMotor.getEncoder().getPosition(), null);
    }

    public class VisionPoseMeasurement
    {
        public double BeganComputingAt;
        public Pose2d Pose;

        public VisionPoseMeasurement(Pose2d pose, double beganComputingAt)
        {
            this.Pose = pose;
            this.BeganComputingAt = beganComputingAt;
        }
    }

}
