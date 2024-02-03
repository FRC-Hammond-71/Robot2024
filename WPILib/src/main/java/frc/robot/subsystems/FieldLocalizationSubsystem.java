package frc.robot.subsystems;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Controllers;
import frc.robot.LimelightHelpers;

/**
 * The localization subsystem utilizes several sensors on the Robot to estimate where the Robot is on the field.
 * 
 * <p>The <code>LocalizationSubsystem</code> is required to be updated after the <code>DriveSubsystem</code>!</p>
 */
public class FieldLocalizationSubsystem extends SubsystemBase 
{
    private DifferentialDrivePoseEstimator PoseEstimator;
    private DriveSubsystem Drive;

    private Pose2d IMUAccumulatedPose = new Pose2d();

    public FieldLocalizationSubsystem(DriveSubsystem drive)
    {
        super();
        // Do not automatically register.
        CommandScheduler.getInstance().unregisterSubsystem(this);

        this.Drive = drive;

        if (RobotBase.isReal())
        {
            this.PoseEstimator = new DifferentialDrivePoseEstimator(this.Drive.Kinematics, Rotation2d.fromDegrees(0), 0, 0, new Pose2d());
        }
        else
        {
            // TODO: Support simulation of Robot
            throw new UnsupportedOperationException("DriveSubsystem cannot be simulated!");
        }

        setDefaultCommand(Commands.run(() -> 
        {            
            if (Controllers.DriverController.getLeftBumperPressed())
            {
                System.out.println("Reset initial position!");

                this.ResetPosition(new Pose2d(14.6, 5.64, Rotation2d.fromDegrees(0)));
            }
        }, this));
    }

    public Pose2d GetEstimatedPose()
    {
        return this.PoseEstimator.getEstimatedPosition();
    }

    public void ResetPosition(Pose2d initialPosition)
    {
        this.Drive.ResetSensors();
        this.IMUAccumulatedPose = initialPosition;
        this.Drive.IMU.reset();
        // this.Drive.IMU.setAngleAdjustment(initialPosition.getRotation().getDegrees());
		this.PoseEstimator.resetPosition(initialPosition.getRotation(), 0, 0, initialPosition);
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
        var wheelPositions = this.Drive.GetWheelPositions();
        
        this.PoseEstimator.update(Rotation2d.fromDegrees(this.Drive.IMU.getAngle()), wheelPositions.leftMeters, wheelPositions.rightMeters);
        
        if (HasVisionPosition())
        {
            var visionMeasurement = this.GetVisionPosition();
            
            this.PoseEstimator.addVisionMeasurement(visionMeasurement.Pose, visionMeasurement.BeganComputingAt);
        }

        {
            // Update using IMU
            var x = this.Drive.IMU.getWorldLinearAccelX();
            var y = this.Drive.IMU.getWorldLinearAccelY();
            var z = this.Drive.IMU.getWorldLinearAccelZ(); // Heading
                       
            
        }
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
