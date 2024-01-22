package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;

public class MovementSubsystem extends SubsystemBase {

    // ------------------ Wheel Specs ---------------------------
    // Radius = 3 in
    // Diameter = Radius * 2 = 6 in
    // Circumference = Diameter * PI = 18.85 inches = 0.48 meters
    // 10.7:1 rotation (Motor makes 10.7 rotations to one wheel turn?)
    // ----------------------------------------------------------

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
    private PIDController LeftMotorsPID = new PIDController(5, 0.25, 0.5);
    private PIDController RightMotorsPID = new PIDController(5, 0.25, 0.5);

    // -----------
    // Controllers
    // -----------
    private DifferentialDrive Drive = new DifferentialDrive(LeftLeadMotor, RightLeadMotor);

    private DifferentialDriveKinematics Kinematics = new DifferentialDriveKinematics(20);

    private DifferentialDrivePoseEstimator PoseEstimator;

    private AHRS IMU = new AHRS(SPI.Port.kMXP);

    /**
     * The desired speed and rotation the robot should be moving at.
     */
    private ChassisSpeeds TargetSpeeds = new ChassisSpeeds();

    public MovementSubsystem() {
        super();

        /*
         * m_armMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
         * m_armMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
         * 
         * m_armMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward,980);
         * m_armMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,-980);
         */

        this.LeftFollowMotor.follow(this.LeftLeadMotor);
        this.RightFollowMotor.follow(this.RightLeadMotor);

        // Set the initial arm position to the current position.
        // TODO: Turn into degrees!!!!!!!!!!!!!!!!!!!!!!!

        this.PoseEstimator = new DifferentialDrivePoseEstimator(
                this.Kinematics,
                new Rotation2d(Units.degreesToRadians(this.IMU.getAngle())),
                0,
                0,
                LimelightHelpers.getBotPose2d("limelight"));
    }

    // ------------------
    // Movement Reporting
    // ------------------
    /**
     * @return The Left Motor Speed in Meters / Second.
     */
    public double GetLeftWheelSpeed() {
        return this.LeftLeadMotor.getEncoder().getVelocity() / 60 * 0.48 * 10.7;
    }

    /**
     * @return The Right Motor Speed in Meters / Second.
     */
    public double GetRightWheelSpeed() {
        return this.RightLeadMotor.getEncoder().getVelocity() / 60 * 0.48 * 10.7;
    }

    /**
     * @return The estimated position of the Robot in Meters.
     */
    public Pose2d GetEstimatedPose() {
        return this.PoseEstimator.getEstimatedPosition();
    }

    public void Drive(ChassisSpeeds speeds) {
        this.TargetSpeeds = speeds;
    }

    public void Stop() {
        this.TargetSpeeds = new ChassisSpeeds();
        this.Drive.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("IMU", String.format("Yaw: %.2f Pitch: %.2f Roll: %.2f",
                this.IMU.getAngle(),
                this.IMU.getPitch(),
                this.IMU.getRoll()));

        // Use kinematics to calculate desired wheel speeds.
        var desiredWheelSpeeds = this.Kinematics.toWheelSpeeds(this.TargetSpeeds);

        this.PoseEstimator.update(
                new Rotation2d(Units.degreesToRadians(this.IMU.getAngle())),
                this.RightLeadMotor.getEncoder().getPosition() * 0.48 * 10.7,
                this.LeftLeadMotor.getEncoder().getPosition() * 0.48 * 10.7);

        if (RobotBase.isReal()) {
            this.PoseEstimator.addVisionMeasurement(LimelightHelpers.getBotPose2d("limelight"),
                    LimelightHelpers.getLatency_Pipeline("limelight"));
            SmartDashboard.putString("Limelight BotPose", LimelightHelpers.getBotPose2d("limelight").toString());
        }

        {
            var estimatedPosition = this.GetEstimatedPose();
            SmartDashboard.putString("Estimated Position", String.format("(%.2f, %.2f) %.2f°",
                    estimatedPosition.getX(),
                    estimatedPosition.getY(),
                    estimatedPosition.getRotation().getDegrees()));
        }

        // SmartDashboard.putString("Target Speeds", this.TargetSpeeds.toString());
        // SmartDashboard.putString("Wheel Speeds", wheelSpeeds.toString());

        // var leftMotorP = wheelSpeeds.leftMetersPerSecond / 0.48 * 10.7 * 60;
        var leftMotorP = this.LeftMotorsPID.calculate(this.GetLeftWheelSpeed(), desiredWheelSpeeds.leftMetersPerSecond)
                / 0.48 / 10.7 * 60 / 5676;
        // var rightMotorP = wheelSpeeds.rightMetersPerSecond / 0.48 * 10.7 * 60;
        var rightMotorP = this.RightMotorsPID.calculate(this.GetRightWheelSpeed(),
                desiredWheelSpeeds.rightMetersPerSecond) / 0.48 / 10.7 * 60 / 5676;

        SmartDashboard.putNumber("PID LEFT ERROR", this.LeftMotorsPID.getPositionError());
        SmartDashboard.putNumber("PID RIGHT ERROR", this.RightMotorsPID.getPositionError());

        this.Drive.tankDrive(leftMotorP, rightMotorP);

        SmartDashboard.putNumber("Left Speed (M/s)", GetLeftWheelSpeed());
        SmartDashboard.putNumber("Left Desired Speed (M/s)", desiredWheelSpeeds.leftMetersPerSecond);

        SmartDashboard.putNumber("Right Speed (M/s)", GetRightWheelSpeed());
        SmartDashboard.putNumber("Right Desired Speed (M/s)", desiredWheelSpeeds.rightMetersPerSecond);
    }
}