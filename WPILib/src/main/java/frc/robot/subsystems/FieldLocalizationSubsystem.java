package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Controllers;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;

/**
 * The localization subsystem utilizes several sensors on the Robot to estimate where the Robot is on the field.
 * 
 * <p>The <code>LocalizationSubsystem</code> is required to be updated after the <code>DriveSubsystem</code>!</p>
 */
public class FieldLocalizationSubsystem extends SubsystemBase 
{    
    // https://www.chiefdelphi.com/t/considering-the-intricacies-of-autonomous-pathfinding-algorithms-and-the-myriad-of-sensor-fusion-techniques-in-frc-how-might-one-harmonize-the-celestial-dance-of-encoder-ticks-and-gyroscopic-precession/441692/28
    private DifferentialDrivePoseEstimator PoseEstimator;

    // -------
    // Sensors
    // -------
    private AHRS IMU;
    private Timer IMURefreshTimer;
    private Pose2d IMUAccumulatedPose = new Pose2d();
    /**
     * The update rate of the IMU in Seconds.
     */
    private double IMUUpdateRate;

    public FieldLocalizationSubsystem()
    {
        super();
        
        if (RobotBase.isReal())
        {
            this.IMU = new AHRS(SPI.Port.kMXP);;
            this.IMURefreshTimer = new Timer();
            this.IMUUpdateRate = 1 / this.IMU.getActualUpdateRate();
            System.out.printf("IMU Update Rate: %d", this.IMU.getActualUpdateRate());
            
            this.PoseEstimator = new DifferentialDrivePoseEstimator(
                RobotContainer.Drive.Kinematics, 
                Rotation2d.fromDegrees(0), 0, 0, 
                new Pose2d(),
                VecBuilder.fill(0.02, 0.02, 0.01),
                VecBuilder.fill(0.1, 0.1, 0.1));
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
        return RobotBase.isReal() ? this.PoseEstimator.getEstimatedPosition() : RobotContainer.Drive.SimulatedDrive.getPose();
    }

    public void ResetPosition(Pose2d initialPosition)
    {
        RobotContainer.Drive.ResetEncoders();

        if (RobotBase.isReal())
        {
            this.IMU.reset();
            this.IMUAccumulatedPose = initialPosition;
            this.PoseEstimator.resetPosition(initialPosition.getRotation(), 0, 0, initialPosition);
        }
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
        if (!RobotBase.isReal()) 
        {
            Constants.Field.setRobotPose(this.GetEstimatedPose());
            return;
        };

        var wheelPositions = RobotContainer.Drive.GetWheelPositions();
        this.PoseEstimator.update(Rotation2d.fromDegrees(this.IMU.getAngle()), wheelPositions.leftMeters, wheelPositions.rightMeters);
        
        if (HasVisionPosition())
        {
            System.out.println("Updating Vision Measurements...");

            var visionMeasurement = this.GetVisionPosition();
            
            this.PoseEstimator.addVisionMeasurement(visionMeasurement.Pose, visionMeasurement.BeganComputingAt);

            Constants.Field.getObject("Robot - Vision").setPose(visionMeasurement.Pose);
        }

        if (this.IMURefreshTimer.hasElapsed(this.IMUUpdateRate)) 
        {
            System.out.printf("Updating IMU Measurements at %d ...", this.IMU.getLastSensorTimestamp());

            // TODO: Use the difference between the current time and getLastSensorTimestamp() to interpolate veleocity. 

            // One G equates to One M/s^2
            // Then scale velocity values by the duration (seconds) passed since last refresh. 
            double deltaX = this.IMU.getVelocityX() * this.IMURefreshTimer.get();
            double deltaY = this.IMU.getVelocityY() * this.IMURefreshTimer.get();
            double deltaRotation = this.IMU.getVelocityZ() * this.IMURefreshTimer.get() / Constants.Drivetrain.TrackCircumference;
            // For the Robot to do a full rotation it must rotate 3.42917278846 meters.

            this.IMUAccumulatedPose = new Pose2d(
                this.IMUAccumulatedPose.getX() + deltaX,
                this.IMUAccumulatedPose.getY() + deltaY,
                this.IMUAccumulatedPose.getRotation().plus(Rotation2d.fromDegrees(deltaRotation * 360))
            );

            // NOTE: May need to use Pose2d.transformBy?

            this.IMURefreshTimer.reset();

            // TODO: Contribute IMU Accumulated Position to PoseEstimator!

            Constants.Field.getObject("Robot - IMU").setPose(this.IMUAccumulatedPose);
        }

        Constants.Field.setRobotPose(this.GetEstimatedPose());
    }

    @Override
    public void simulationPeriodic() 
    {
        
    }

    @Override
    public void initSendable(SendableBuilder builder) 
    {
        if (RobotBase.isReal())
        {
            builder.addDoubleProperty("IMU Update Rate", () -> Units.secondsToMilliseconds(this.IMUUpdateRate), null);
        }

        builder.addStringProperty("Estimated Position", () -> {

            var estimatedPose = this.GetEstimatedPose();
            return String.format("X: %.2f Y: %.2f Heading: %.2f", estimatedPose.getX(), estimatedPose.getY(), estimatedPose.getRotation().getDegrees());

        }, null);
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
