package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import java.util.Date;

public class DriveSubsystem extends SubsystemBase {

    // ------
    // Motors
    // ------ 
    private CANSparkMax LeftLeadMotor = new CANSparkMax(5,MotorType.kBrushless);
    private CANSparkMax RightLeadMotor = new CANSparkMax(4,MotorType.kBrushless);
    // private CANSparkMax LeftFollowMotor = new CANSparkMax(3,MotorType.kBrushless);
    // private CANSparkMax RightFollowMotor =  new CANSparkMax(2,MotorType.kBrushless);

    public DifferentialDrive Drive = new DifferentialDrive(LeftLeadMotor, RightLeadMotor);
    // https://docs.wpilib.org/en/stable/docs/software/hardware-apis/motors/wpi-drive-classes.html

    private DifferentialDriveKinematics Kinematics = new DifferentialDriveKinematics(20);

    /**
     * The desired speed and rotation the robot should be moving at.
     */
    private ChassisSpeeds TargetSpeeds = new ChassisSpeeds();


    public DriveSubsystem()
    {
        super();

        // this.RightLeadMotor.setInverted(true);

        // this.Drive.setDeadband(0.05);
    }

    // ------------------
    // Movement Reporting
    // ------------------
    /**
     * @return The Left Motor Speed in Meters / Second.
     */
    public double GetLeftWheelSpeed()
    {
        return this.LeftLeadMotor.getEncoder().getVelocity() / 60 * 0.48 / 10.7;
    }
     /**
     * @return The Right Motor Speed in Meters / Second.
     */
    public double GetRightWheelSpeed()
    {
        return this.RightLeadMotor.getEncoder().getVelocity() / 60 * 0.48 / 10.7;
    }

    public ChassisSpeeds GetOdometry()
    {
        // Wheel Specs:
        // Radius = 3 in
        // Diameter = Radius * 2 = 6 in
        // Circumference = Diameter * PI = 18.85 inches = 0.48 meters

        // Requires meters per second.
        var differential_drive_wheel_speeds = new DifferentialDriveWheelSpeeds(
            GetLeftWheelSpeed(),
            GetRightWheelSpeed()
        );

        // 10.7:1 rotation (Motor makes 10.7 rotations to one wheel turn)

        return this.Kinematics.toChassisSpeeds(differential_drive_wheel_speeds);
    }

    public void Drive(ChassisSpeeds speeds)
    {
        // Combine the current speeds, and given speeds.
        // this.TargetSpeeds = new ChassisSpeeds(
        //     speeds.vxMetersPerSecond - this.TargetSpeeds.vxMetersPerSecond,
        //     speeds.vyMetersPerSecond - this.TargetSpeeds.vyMetersPerSecond,
        //     speeds.omegaRadiansPerSecond - this.TargetSpeeds.omegaRadiansPerSecond
        // );
        this.TargetSpeeds = speeds;
    }

    public void Stop()
    {
        this.TargetSpeeds = new ChassisSpeeds();
        this.Drive.stopMotor();
    }

    @Override
    public void periodic() 
    {
        SmartDashboard.putString("GetOdometry", GetOdometry().toString());
        
        var turning_error = GetOdometry().omegaRadiansPerSecond / this.TargetSpeeds.omegaRadiansPerSecond;

        var wheelSpeeds = this.Kinematics.toWheelSpeeds(this.TargetSpeeds);

        SmartDashboard.putString("Target  Speeds", this.TargetSpeeds.toString());
        SmartDashboard.putString("Wheel Speeds", wheelSpeeds.toString());

        var leftMotorP = wheelSpeeds.leftMetersPerSecond / 0.48 * 10.7 * 60;
        var rightMotorP = wheelSpeeds.rightMetersPerSecond / 0.48 * 10.7 * 60;

        SmartDashboard.putNumber("TURNING_ERROR", turning_error);

        this.Drive.tankDrive(leftMotorP / 5676, rightMotorP / 5676);

    
        SmartDashboard.putString("Left Speed", String.format("%f RPM", GetLeftWheelSpeed()));
        SmartDashboard.putString("Right Speed", String.format("%f RPM", GetRightWheelSpeed()));
    }
}