package frc.robot.subsystems.Movement;

import java.io.Console;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.utilities.ChassisSpeedsUtils;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;

public class ActualDriveSubsystem extends DriveSubsystem {

    // ------
    // Motors
    // ------
    private CANSparkMax LeftLeadMotor = new CANSparkMax(5, MotorType.kBrushless);
    private CANSparkMax RightLeadMotor = new CANSparkMax(4, MotorType.kBrushless);
    private CANSparkMax LeftFollowMotor = new CANSparkMax(3, MotorType.kBrushless);
    private CANSparkMax RightFollowMotor = new CANSparkMax(2, MotorType.kBrushless);
    // ---------
    // Motor PID (These need tuning!)
    // ---------
    private PIDController LeftMotorsPID = new PIDController(0.1, 0, 0);
    private PIDController RightMotorsPID = new PIDController(0.1, 0, 0);

    private SimpleMotorFeedforward FeedForward = new SimpleMotorFeedforward(0.10158, 2.161, 0.53799);

    // -----------
    // Controllers
    // -----------
    // private DifferentialDrive Drive = new DifferentialDrive(LeftLeadMotor,
    // RightLeadMotor);

    private DifferentialDriveKinematics Kinematics = new DifferentialDriveKinematics(Constants.Drivetrain.TrackWidth);

    private DifferentialDrivePoseEstimator PoseEstimator;

    private AHRS IMU = new AHRS(SPI.Port.kMXP);

    /**
     * The desired speed and rotation the robot should be moving at.
     */
    private ChassisSpeeds TargetSpeeds = new ChassisSpeeds();
    private DifferentialDriveWheelSpeeds PreviousWheelSpeeds = new DifferentialDriveWheelSpeeds();

    public ActualDriveSubsystem() {
        super();

        this.LeftFollowMotor.follow(this.LeftLeadMotor);
        this.RightFollowMotor.follow(this.RightLeadMotor);

        AutoBuilder.configureRamsete(
                this::GetEstimatedPose,
                // NOTE: May not work?
                (pose) -> this.PoseEstimator.resetPosition(new Rotation2d(0), 0, 0, pose),
                this::GetChassisSpeeds,
                this::Drive,
                new ReplanningConfig(),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this);

        this.LeftLeadMotor.getEncoder().setPosition(0);
        this.RightLeadMotor.getEncoder().setPosition(0);

        this.PoseEstimator = new DifferentialDrivePoseEstimator(
                this.Kinematics,
                new Rotation2d(Units.degreesToRadians(-this.IMU.getAngle())),
                0,
                0,
                LimelightHelpers.getBotPose2d("limelight"));

        Shuffleboard.getTab("Drive").add(this.LeftMotorsPID);
        Shuffleboard.getTab("Drive").add(this.RightMotorsPID);
        Shuffleboard.getTab("Drive").addDouble("Feed Forward",
                () -> this.FeedForward.calculate(this.TargetSpeeds.vxMetersPerSecond));

    }

    // ------------------
    // Movement Reporting
    // ------------------
    /**
     * @return The Left Motor Speed in Meters / Second.
     */
    @Override
    public double GetLeftWheelSpeed() {
        return this.LeftLeadMotor.getEncoder().getVelocity() / 60 * Constants.Drivetrain.WheelCircumference
                / Constants.Drivetrain.WheelGearing;
    }

    /**
     * @return The Right Motor Speed in Meters / Second.
     */
    @Override
    public double GetRightWheelSpeed() {
        return this.RightLeadMotor.getEncoder().getVelocity() / 60 * Constants.Drivetrain.WheelCircumference
                / Constants.Drivetrain.WheelGearing;
    }

    public double GetLeftWheelTravel() {
        return this.LeftLeadMotor.getEncoder().getPosition()
                * Constants.Drivetrain.WheelCircumference
                / Constants.Drivetrain.WheelGearing;
    }

    public double GetRightWheelTravel() {
        return this.RightLeadMotor.getEncoder().getPosition()
                * Constants.Drivetrain.WheelCircumference
                / Constants.Drivetrain.WheelGearing;
    }

    /**
     * @return The estimated position of the Robot in Meters.
     */
    @Override
    public Pose2d GetEstimatedPose() {
        return this.PoseEstimator.getEstimatedPosition();
    }

    @Override
    public void Drive(ChassisSpeeds speeds) {
        this.TargetSpeeds = speeds;
    }

    @Override
    public void Stop() {
        this.TargetSpeeds = new ChassisSpeeds();
        this.LeftLeadMotor.stopMotor();
        this.RightLeadMotor.stopMotor();
        // this.Drive.stopMotor();
    }

    @Override
    public void periodic() {

        this.TargetSpeeds = ChassisSpeedsUtils.Clamp(TargetSpeeds, 2, 0, Units.degreesToRadians(200));
        
        SmartDashboard.putString("IMU", String.format("Yaw: %.2f Pitch: %.2f Roll: %.2f",
        this.IMU.getAngle(),
        this.IMU.getPitch(),
        this.IMU.getRoll()));
        
        // Use kinematics to calculate desired wheel speeds.
        var desiredWheelSpeeds = this.Kinematics.toWheelSpeeds(this.TargetSpeeds);
        
        SmartDashboard.putNumber("Left Motor Inaccuracy", Math.abs(PreviousWheelSpeeds.leftMetersPerSecond - this.GetLeftWheelSpeed()));
        SmartDashboard.putNumber("Right Motor Inaccuracy", Math.abs(PreviousWheelSpeeds.rightMetersPerSecond - this.GetRightWheelSpeed()));
        
        this.PreviousWheelSpeeds = desiredWheelSpeeds;
        
        SmartDashboard.putString("Desired Movement", String.format("Forward: %.2f (M/s) Rot: %.2f (D/s)", this.TargetSpeeds.vxMetersPerSecond, Units.radiansToDegrees(this.TargetSpeeds.omegaRadiansPerSecond)));


        this.PoseEstimator.update(
                new Rotation2d(Units.degreesToRadians(this.IMU.getAngle())),
                this.GetLeftWheelTravel(),
                this.GetRightWheelTravel());

        SmartDashboard.putString("Limelight", LimelightHelpers.getBotPose2d("limelight").toString());

        this.PoseEstimator.addVisionMeasurement(
                LimelightHelpers.getBotPose2d("limelight"),
                LimelightHelpers.getLatency_Pipeline("limelight"));

        this.Field.setRobotPose(this.PoseEstimator.getEstimatedPosition());

        // System.out.println(this.LeftLeadMotor.getEncoder().getPosition());

        // SmartDashboard.putString("Limelight BotPose", LimelightHelpers.getBotPose2d("limelight").toString());

        {
            var estimatedPosition = this.GetEstimatedPose();
            SmartDashboard.putString("Estimated Position", String.format("(%.2f, %.2f) %.2f°",
                    estimatedPosition.getX(),
                    estimatedPosition.getY(),
                    estimatedPosition.getRotation().getDegrees()));
        }

        this.LeftLeadMotor.setVoltage(
                LeftMotorsPID.calculate(this.GetLeftWheelSpeed(), desiredWheelSpeeds.leftMetersPerSecond)
                        + this.FeedForward.calculate(desiredWheelSpeeds.leftMetersPerSecond));
        this.RightLeadMotor.setVoltage(
                RightMotorsPID.calculate(this.GetRightWheelSpeed(), desiredWheelSpeeds.rightMetersPerSecond)
                        + this.FeedForward.calculate(desiredWheelSpeeds.rightMetersPerSecond));

        SmartDashboard.putNumber("Left Speed (M/s)", GetLeftWheelSpeed());
        SmartDashboard.putNumber("Left Desired Speed (M/s)", desiredWheelSpeeds.leftMetersPerSecond);

        SmartDashboard.putNumber("Right Speed (M/s)", GetRightWheelSpeed());
        SmartDashboard.putNumber("Right Desired Speed (M/s)", desiredWheelSpeeds.rightMetersPerSecond);
        
        SmartDashboard.putNumber("PID LEFT ERROR", this.LeftMotorsPID.getPositionError());
        SmartDashboard.putNumber("PID RIGHT ERROR", this.RightMotorsPID.getPositionError());

    }

    @Override
    public ChassisSpeeds GetChassisSpeeds() {
        return this.TargetSpeeds;
    }
}