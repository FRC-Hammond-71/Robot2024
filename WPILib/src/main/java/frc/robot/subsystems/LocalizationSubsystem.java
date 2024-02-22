package frc.robot.subsystems;

import java.time.Duration;
import java.util.Optional;
import java.util.logging.Logger;

import org.photonvision.EstimatedRobotPose;
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
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.FunctionalPeriodic;
import frc.IPeriodic;
import frc.RobotSubsystem;
import frc.robot.Constants;
import frc.robot.Controllers;
import frc.robot.ElapsedTimer;
import frc.robot.FieldGeometry;
import frc.robot.Robot;
import frc.robot.RobotContainer;

/**
 * The localization subsystem utilizes several sensors on the Robot to estimate
 * where the Robot is on the field.
 * 
 * <p>
 * The <code>LocalizationSubsystem</code> is required to be updated after the
 * <code>DriveSubsystem</code>!
 * </p>
 */
public class LocalizationSubsystem extends RobotSubsystem<Robot>
{
    
    // https://www.chiefdelphi.com/t/considering-the-intricacies-of-autonomous-pathfinding-algorithms-and-the-myriad-of-sensor-fusion-techniques-in-frc-how-might-one-harmonize-the-celestial-dance-of-encoder-ticks-and-gyroscopic-precession/441692/28
    private DifferentialDrivePoseEstimator PoseEstimator;
    
    // https://www.team254.com/resources/
    // https://drive.google.com/drive/folders/1tZ6xdn-xzFpcAzW3VB5ttzQwuhUmvPJB
    
    // -------
    // Sensors
    // -------
    private AHRS IMU;
    private ElapsedTimer IMUTimer;
    private Pose2d IMUAccumulatedPose = new Pose2d();
    
    // -------
    // Cameras
    // -------
    private PhotonCamera LauncherCamera, IntakeCamera;
    
    // https://docs.photonvision.org/en/latest/docs/programming/photonlib/robot-pose-estimator.html
    private PhotonPoseEstimator LauncherCameraPoseEstimator, IntakeCameraPoseEstimator;
    
    /**
     * Wether or not vision has contributed to the absolute position.
     * If not, the first measurement from vision will be used to rest the pose
     * estimator.
     */
    private boolean HasAbsolutePositionFixed = false;

    public IPeriodic RelativeSensorUpdatePeriodic;
    public IPeriodic VisionUpdatePeriodic;

    public LocalizationSubsystem(Robot robot)
    {
        super(robot);

        this.RelativeSensorUpdatePeriodic = new FunctionalPeriodic(() -> 
        {
            if (RobotBase.isSimulation()) return;

            // Read sensors such as the drive encoders and IMU at 200 Hz
            this.UpdatePoseEstimationUsingWheels();
            this.UpdatePoseEstimationUsingIMU();

        }, Duration.ofMillis(5));

        this.VisionUpdatePeriodic = new FunctionalPeriodic(() ->
        {
            if (RobotBase.isSimulation()) return;

            // Update field-position vision at 60 Hz
            this.UpdatePoseEstimationUsingVision();

        }, Duration.ofMillis(60));
    }
    
    @Override
    protected void initializeReal()
    {
        this.IMU = new AHRS(SPI.Port.kMXP);

        this.IMUTimer = new ElapsedTimer(Duration.ofSeconds(1 / this.IMU.getActualUpdateRate()));
        System.out.printf("IMU Update Rate: %d", this.IMU.getActualUpdateRate());

        this.PoseEstimator = new DifferentialDrivePoseEstimator(
                Robot.Drive.Kinematics,
                Rotation2d.fromDegrees(0), 0, 0,
                new Pose2d(),
                VecBuilder.fill(0.02, 0.02, 0.01),
                VecBuilder.fill(0.1, 0.1, 0.1));

        this.LauncherCamera = new PhotonCamera("Sauron");
        this.IntakeCamera = new PhotonCamera("Roz");

        this.IntakeCameraPoseEstimator = new PhotonPoseEstimator(
                AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
                PoseStrategy.MULTI_TAG_PNP_ON_RIO,
                this.IntakeCamera,
                new Transform3d());

        this.LauncherCameraPoseEstimator = new PhotonPoseEstimator(
                AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
                PoseStrategy.MULTI_TAG_PNP_ON_RIO,
                this.LauncherCamera,
                new Transform3d());
    }

    public Pose2d GetEstimatedPose()
    {
        var pose = RobotBase.isReal() 
            ? this.PoseEstimator.getEstimatedPosition()
            : Robot.Drive.SimulatedDrive.getPose();

        return new Pose2d(
            pose.getTranslation(),
            Rotation2d.fromRadians(MathUtil.inputModulus(pose.getRotation().getRadians(), -Math.PI, Math.PI)));
    }

    public void ResetPosition(Pose2d position)
    {
        Robot.Drive.ResetEncoders();

        if (RobotBase.isReal())
        {
            // Reset Yaw Gyro
            this.IMU.reset();
            this.IMUAccumulatedPose = position;
            this.PoseEstimator.resetPosition(position.getRotation(), 0, 0, position);
        }
    }

    private synchronized void ApplyVisionMeasurement(Optional<EstimatedRobotPose> robotPose)
    {
        if (robotPose.isEmpty() || RobotBase.isSimulation()) return;
    
        // If an absolute field-relative position has not been reported until now, then reset position.
        if (this.HasAbsolutePositionFixed)
        {
            this.PoseEstimator.addVisionMeasurement(
                robotPose.get().estimatedPose.toPose2d(),
                robotPose.get().timestampSeconds);
        }
        else 
        {
            this.ResetPosition(robotPose.get().estimatedPose.toPose2d());

            // DataLogManager.getLog().
            this.HasAbsolutePositionFixed = true;
        }
    }

    @Override
    public void periodic()
    {
        super.periodic();

        Constants.Field.getObject("Speaker Left").setPose(new Pose2d(FieldGeometry.GetSpeakerIntakeLeftMostPosition(), new Rotation2d()));
        Constants.Field.getObject("Speaker Right").setPose(new Pose2d(FieldGeometry.GetSpeakerIntakeRightMostPosition(), new Rotation2d()));
        // System.out.println(allowedError);
    }

    @Override
    public void simulationPeriodic()
    {
        Constants.Field.setRobotPose(this.GetEstimatedPose());
    }

    protected void UpdatePoseEstimationUsingWheels()
    {
        var wheelPositions = Robot.Drive.GetWheelPositions();

        this.PoseEstimator.update(
            Rotation2d.fromDegrees(this.IMU.getAngle()),
            wheelPositions.leftMeters,
            wheelPositions.rightMeters);
    }

    protected void UpdatePoseEstimationUsingIMU()
    {
        // System.out.printf("Updating IMU Measurements at %d\n", this.IMU.getLastSensorTimestamp());

        double deltaX = this.IMU.getVelocityX() * this.IMUTimer.Timer.get();
        double deltaY = this.IMU.getVelocityY() * this.IMUTimer.Timer.get();
        // double deltaRotation = this.IMU.getVelocityZ() * this.IMUTimer.Timer.get();
        // double deltaRotation = this.IMU.getVelocityZ() * this.IMUTimer.Timer.get()
        //         / Constants.Drivetrain.TrackCircumference;

        SmartDashboard.putNumber("IMU - DeltaX", deltaX);
        SmartDashboard.putNumber("IMU - DeltaY", deltaY);
        // SmartDashboard.putNumber("IMU - Delta Rotation", deltaRotation);

        this.IMUAccumulatedPose = this.IMUAccumulatedPose.transformBy(new Transform2d(
            new Translation2d(deltaX, deltaY),

            // TODO: Use yaw
            Rotation2d.fromDegrees(0)
        ));

        // TODO: Contribute IMU Accumulated Position to PoseEstimator!

        Constants.Field.getObject("Robot - IMU").setPose(this.IMUAccumulatedPose);
    }

    protected void UpdatePoseEstimationUsingVision()
    {
        // var fieldPoseFromLauncher = this.LauncherCameraPoseEstimator.update();
        // if (fieldPoseFromLauncher.isPresent())
        // {
        //     this.ApplyVisionMeasurement(fieldPoseFromLauncher);

        //     Constants.Field.getObject("Robot - Launcher Vision").setPose(fieldPoseFromLauncher.get().estimatedPose.toPose2d());
        // }

        // var fieldPoseFromIntake = this.IntakeCameraPoseEstimator.update();
        // if (fieldPoseFromIntake.isPresent())
        // {
        //     this.ApplyVisionMeasurement(fieldPoseFromLauncher);

        //     Constants.Field.getObject("Robot - Intake Vision").setPose(fieldPoseFromIntake.get().estimatedPose.toPose2d());
        // }
    }

    @Override
    protected void realPeriodic()
    {
        // this.UpdatePoseEstimationUsingWheels();
        // this.UpdatePoseEstimationUsingIMU();
        // this.UpdatePoseEstimationUsingVision();

        // Update smart-dashboard with Robot positioning at default 20 Hz
        Constants.Field.setRobotPose(this.GetEstimatedPose());
    }

    @Override
    public void initSendable(SendableBuilder builder)
    {
        if (RobotBase.isReal())
        {
            builder.addDoubleProperty("IMU Update Rate", () -> this.IMUTimer.Period.toMillis(), null);

            builder.addBooleanProperty("Is IMU Connected", () -> this.IMU.isConnected(), null);
        }

        builder.addStringProperty("Estimated Position", () ->
        {
            var estimatedPose = this.GetEstimatedPose();
            return String.format("X: %.2f Y: %.2f Heading: %.2f", estimatedPose.getX(), estimatedPose.getY(),
                    estimatedPose.getRotation().getRadians());
        }, null);
    }
}
