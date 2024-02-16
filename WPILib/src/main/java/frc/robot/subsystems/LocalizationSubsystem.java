package frc.robot.subsystems;

import java.time.Duration;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Controllers;
import frc.robot.ElapsedTimer;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;

/**
 * The localization subsystem utilizes several sensors on the Robot to estimate where the Robot is on the field.
 * 
 * <p>The <code>LocalizationSubsystem</code> is required to be updated after the <code>DriveSubsystem</code>!</p>
 */
public class LocalizationSubsystem extends SubsystemBase 
{    
    // https://www.chiefdelphi.com/t/considering-the-intricacies-of-autonomous-pathfinding-algorithms-and-the-myriad-of-sensor-fusion-techniques-in-frc-how-might-one-harmonize-the-celestial-dance-of-encoder-ticks-and-gyroscopic-precession/441692/28
    private DifferentialDrivePoseEstimator PoseEstimator;

    // -------
    // Sensors
    // -------
    private AHRS IMU;
    private ElapsedTimer IMUTimer;
    private Pose2d IMUAccumulatedPose = new Pose2d();

    // -------
    // Cameras
    // -------
    private PhotonCamera LauncherCamera;
    // private PhotonCamera LauncherCamera, IntakeCamera;

    // https://docs.photonvision.org/en/latest/docs/programming/photonlib/robot-pose-estimator.html
    private PhotonPoseEstimator LauncherCameraPoseEstimator;
    // private PhotonPoseEstimator LauncherCameraPoseEstimator, IntakeCameraPoseEstimator;

    public LocalizationSubsystem()
    {
        super();
        
        if (RobotBase.isReal())
        {
            this.IMU = new AHRS(SPI.Port.kMXP);

            this.IMUTimer = new ElapsedTimer(Duration.ofSeconds(1 / this.IMU.getActualUpdateRate()));
            System.out.printf("IMU Update Rate: %d", this.IMU.getActualUpdateRate());
            
            this.PoseEstimator = new DifferentialDrivePoseEstimator(
                RobotContainer.Drive.Kinematics, 
                Rotation2d.fromDegrees(0), 0, 0, 
                new Pose2d(),
                VecBuilder.fill(0.02, 0.02, 0.01),
                VecBuilder.fill(0.1, 0.1, 0.1));

            this.LauncherCamera = new PhotonCamera("launcher");
            // this.IntakeCamera = new PhotonCamera("intake");

            // TODO: Update offsets.
            // this.IntakeCameraPoseEstimator = new PhotonPoseEstimator(
            //     AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(), 
            //     PoseStrategy.MULTI_TAG_PNP_ON_RIO,
            //     this.IntakeCamera,
            //     new Transform3d());

            this.LauncherCameraPoseEstimator = new PhotonPoseEstimator(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(), 
                PoseStrategy.MULTI_TAG_PNP_ON_RIO, 
                this.LauncherCamera,
                new Transform3d());
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
        var pose = RobotBase.isReal() ? this.PoseEstimator.getEstimatedPosition() : RobotContainer.Drive.SimulatedDrive.getPose();
        
        return new Pose2d(pose.getTranslation(), new Rotation2d(MathUtil.inputModulus(pose.getRotation().getRadians(), -Math.PI, Math.PI)));
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

    @Override
    public void periodic() 
    {        
        if (!RobotBase.isReal()) 
        {
            // If Robot is being simulated, do not do actual calculations and exit.
            Constants.Field.setRobotPose(this.GetEstimatedPose());
            return;
        };

        var wheelPositions = RobotContainer.Drive.GetWheelPositions();
        this.PoseEstimator.update(Rotation2d.fromDegrees(this.IMU.getAngle()), wheelPositions.leftMeters, wheelPositions.rightMeters);
        
        //#region Vision Measurements
        var optionalFieldPoseFromLauncher = this.LauncherCameraPoseEstimator.update();
        if (optionalFieldPoseFromLauncher.isPresent())
        {
            var fieldPoseResult = optionalFieldPoseFromLauncher.get();
            var fieldPose2d = fieldPoseResult.estimatedPose.toPose2d();

            this.PoseEstimator.addVisionMeasurement(
                fieldPose2d, 
                fieldPoseResult.timestampSeconds);

            Constants.Field.getObject("Robot - Launcher Vision").setPose(fieldPose2d);
        }
        //#endregion

        if (this.IMUTimer.hasElapsed()) 
        {
            // System.out.printf("Updating IMU Measurements at %d ...", this.IMU.getLastSensorTimestamp());

            // TODO: Use the difference between the current time and getLastSensorTimestamp() to interpolate veleocity. 

            // One G equates to One M/s^2
            // Then scale velocity values by the duration (seconds) passed since last refresh. 
            double deltaX = this.IMU.getVelocityX() * this.IMUTimer.Timer.get();
            double deltaY = this.IMU.getVelocityY() * this.IMUTimer.Timer.get();
            double deltaRotation = this.IMU.getVelocityZ() * this.IMUTimer.Timer.get() / Constants.Drivetrain.TrackCircumference;

            SmartDashboard.putNumber("IMU - DeltaX", deltaX);
            SmartDashboard.putNumber("IMU - DeltaY", deltaY);
            SmartDashboard.putNumber("IMU - Delta Rotation", deltaRotation);

            this.IMUAccumulatedPose = new Pose2d(
                this.IMUAccumulatedPose.getX() + deltaX,
                this.IMUAccumulatedPose.getY() + deltaY,
                this.IMUAccumulatedPose.getRotation().plus(Rotation2d.fromDegrees(deltaRotation * 360))
            );

            // NOTE: May need to use Pose2d.transformBy?

            this.IMUTimer.Timer.reset();

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
            builder.addDoubleProperty("IMU Update Rate", () -> this.IMUTimer.Period.toMillis(), null);

            builder.addBooleanProperty("Is IMU Connected", () -> this.IMU.isConnected(), null);
        }

        builder.addStringProperty("Estimated Position", () -> {

            var estimatedPose = this.GetEstimatedPose();
            return String.format("X: %.2f Y: %.2f Heading: %.2f", estimatedPose.getX(), estimatedPose.getY(), estimatedPose.getRotation().getRadians());

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
