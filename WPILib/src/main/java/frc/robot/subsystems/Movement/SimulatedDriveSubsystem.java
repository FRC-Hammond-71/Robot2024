package frc.robot.subsystems.Movement;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.utilities.ChassisSpeedsUtils;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;

public class SimulatedDriveSubsystem extends DriveSubsystem {

    private DifferentialDrivetrainSim Drive = new DifferentialDrivetrainSim(
        DCMotor.getNEO(2),
        Constants.Drivetrain.WheelGearing,
        (double) 7.5, // Random value
        (double) 90,
        Constants.Drivetrain.WheelRadius,
        Constants.Drivetrain.TrackWidth,
       
        // The standard deviations for measurement noise:
        // x and y:          0.001 m
        // heading:          0.001 rad
        // l and r velocity: 0.1   m/s
        // l and r position: 0.005 m
        VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
        // VecBuilder.fill(0, 0, 0, 0, 0, 0, 0)

    );

    // ---------
    // Motor PID (These need tuning!)
    // ---------
    private PIDController LeftMotorsPID = new PIDController(5, 0.25, 0.5);
    private PIDController RightMotorsPID = new PIDController(5, 0.25, 0.5);

    private SimpleMotorFeedforward FeedForward = new SimpleMotorFeedforward(0.10158, 2.161, 0.53799);

    // -----------
    // Controllers
    // -----------
    // private DifferentialDrive Drive = new DifferentialDrive(LeftLeadMotor, RightLeadMotor);

    private DifferentialDriveKinematics Kinematics = new DifferentialDriveKinematics(Constants.Drivetrain.TrackWidth);

    private DifferentialDrivePoseEstimator PoseEstimator;

    /**
     * The desired speed and rotation the robot should be moving at.
     */
    private ChassisSpeeds TargetSpeeds = new ChassisSpeeds();
    private DifferentialDriveWheelSpeeds PreviousWheelSpeeds = new DifferentialDriveWheelSpeeds();

    public SimulatedDriveSubsystem() 
    {
        super();

        AutoBuilder.configureRamsete(
            this::GetEstimatedPose,
            this::ResetPose,
            this::GetChassisSpeeds,
            this::Drive,
            new ReplanningConfig(),
            () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
            },
            this
        );

        this.PoseEstimator = new DifferentialDrivePoseEstimator(this.Kinematics,
            this.Drive.getHeading(),
            0,
            0,
            new Pose2d()
        );

        Shuffleboard.getTab("Drive").add(this.LeftMotorsPID);
        Shuffleboard.getTab("Drive").add(this.RightMotorsPID);
        Shuffleboard.getTab("Drive").addDouble("Feed Forward", () -> this.FeedForward.calculate(this.TargetSpeeds.vxMetersPerSecond));
        
        Shuffleboard.getTab("Drive").add("Go to Origin", this.PathFindToPose(
            new Pose2d(),
            new PathConstraints(2, 3, 0.2, 3)
        ));
    }

    // ------------------
    // Movement Reporting
    // ------------------
    /**
     * @return The Left Motor Speed in Meters / Second.
     */
    @Override
    public double GetLeftWheelSpeed() {
        return this.Drive.getLeftVelocityMetersPerSecond();
    }

    /**
     * @return The Right Motor Speed in Meters / Second.
     */
    @Override
    public double GetRightWheelSpeed() {
        return this.Drive.getRightVelocityMetersPerSecond();
    }

    public void ResetPose(Pose2d pose)
    {
        this.Drive.setPose(pose);
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
        // this.Drive.stopMotor();
    }

    @Override
    public void simulationPeriodic() {

        // Clamp the movement.
        this.TargetSpeeds = ChassisSpeedsUtils.Clamp(TargetSpeeds, 3, 0, Units.degreesToRadians(100));

        SmartDashboard.putString("IMU", String.format("Yaw: %.2f",
            this.Drive.getHeading().getDegrees()
        ));

        // Use kinematics to calculate desired wheel speeds.
        var desiredWheelSpeeds = this.Kinematics.toWheelSpeeds(this.TargetSpeeds);
        
        this.PoseEstimator.update(
            this.Drive.getHeading(),
            this.Drive.getLeftPositionMeters(),
            this.Drive.getRightPositionMeters()
        );
            
        this.Field.setRobotPose(this.PoseEstimator.getEstimatedPosition());
        
        // this.PoseEstimator.addVisionMeasurement(LimelightHelpers.getBotPose2d("limelight"),
        // LimelightHelpers.getLatency_Pipeline("limelight"));
        // SmartDashboard.putString("Limelight BotPose", LimelightHelpers.getBotPose2d("limelight").toString());
    
        {
            var estimatedPosition = this.GetEstimatedPose();
            SmartDashboard.putString("Estimated Position", String.format("(%.2f, %.2f) %.2f°",
            estimatedPosition.getX(),
            estimatedPosition.getY(),
            estimatedPosition.getRotation().getDegrees()));
        }
        
        this.Drive.update(0.02);
        
        SmartDashboard.putString("Desired Speeds", this.TargetSpeeds.toString());
        
        SmartDashboard.putNumber("Left Motor Inaccuracy", Math.abs(PreviousWheelSpeeds.leftMetersPerSecond - this.GetLeftWheelSpeed()));
        SmartDashboard.putNumber("Right Motor Inaccuracy", Math.abs(PreviousWheelSpeeds.rightMetersPerSecond - this.GetRightWheelSpeed()));
        
        // LeftMotorsPID.calculate(this.GetLeftWheelSpeed(), desiredWheelSpeeds.leftMetersPerSecond);
        // RightMotorsPID.calculate(this.GetRightWheelSpeed(), desiredWheelSpeeds.rightMetersPerSecond);

        // var leftMotorRPM = desiredWheelSpeeds.leftMetersPerSecond / 0.48 * 60 * 10.7;
        // var rightMotorPPM = desiredWheelSpeeds.rightMetersPerSecond / 0.48 * 60 * 10.7;

        // var leftMotorVoltage = (leftMotorRPM / 5676)  * RobotController.getInputVoltage();
        // var rightMotorVoltage = (rightMotorPPM / 5676) * RobotController.getInputVoltage();
    
        // SmartDashboard.putNumber("Left Motor PER", leftMotorRPM / 5676);
        // SmartDashboard.putNumber("Right Motor PER", rightMotorPPM / 5676);
        
        // SmartDashboard.putNumber("PID LEFT ERROR", this.LeftMotorsPID.getPositionError());
        // SmartDashboard.putNumber("PID RIGHT ERROR", this.RightMotorsPID.getPositionError());

        this.Drive.setInputs(
            LeftMotorsPID.calculate(this.GetLeftWheelSpeed(), desiredWheelSpeeds.leftMetersPerSecond) + FeedForward.calculate(desiredWheelSpeeds.leftMetersPerSecond), 
            RightMotorsPID.calculate(this.GetRightWheelSpeed(), desiredWheelSpeeds.rightMetersPerSecond) + FeedForward.calculate(desiredWheelSpeeds.rightMetersPerSecond)
        );
        
        SmartDashboard.putNumber("Left Speed (M/s)", GetLeftWheelSpeed());
        SmartDashboard.putNumber("Left Desired Speed (M/s)", desiredWheelSpeeds.leftMetersPerSecond);
        
        SmartDashboard.putNumber("Right Speed (M/s)", GetRightWheelSpeed());
        SmartDashboard.putNumber("Right Desired Speed (M/s)", desiredWheelSpeeds.rightMetersPerSecond);

        this.PreviousWheelSpeeds = desiredWheelSpeeds;
    }
        
    @Override
    public ChassisSpeeds GetChassisSpeeds() 
    {
        return this.TargetSpeeds;
    }    
}