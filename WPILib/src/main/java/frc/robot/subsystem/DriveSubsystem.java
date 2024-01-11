package frc.robot.subsystem;
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
    private CANSparkMax LeftFollowMotor = new CANSparkMax(3,MotorType.kBrushless);
    private CANSparkMax RightFollowMotor =  new CANSparkMax(2,MotorType.kBrushless);
    private CANSparkMax ArmMotor = new CANSparkMax(6,MotorType.kBrushless);
    /**
     * Intake & Launcher Motor
     */
    public CANSparkMax IntakeMotor = new CANSparkMax(7, MotorType.kBrushless);

    public DifferentialDrive Drive = new DifferentialDrive(LeftLeadMotor, RightLeadMotor);

    public DriveSubsystem() { }

    // ------------------
    // Movement Reporting
    // ------------------
    /**
     * Left Motor Velocity in Seconds.
     */
    public double GetLeftMotorVelocity()
    {
        return this.LeftLeadMotor.getEncoder().getVelocity();
    }
    /**
     * Right Motor Velocity in Seconds.
     */
    public double GetRightMotorVelocity()
    {
        return this.LeftLeadMotor.getEncoder().getVelocity();
    }

    public ChassisSpeeds GetChassisSpeeds()
    {
        var differential_drive_kinematics = new DifferentialDriveKinematics(20);

        // Wheel Specs:
        // Radius = 3 in
        // Diameter = Radius * 2 = 6 in
        // Circumference = Diameter * PI = 18.85 inches = 0.48 meters

        // Requires meters per second.
        var differential_drive_wheel_speeds = new DifferentialDriveWheelSpeeds(
            GetLeftMotorVelocity() / 60 * 0.48,
            GetRightMotorVelocity() / 60 * 0.48
        );

        return differential_drive_kinematics.toChassisSpeeds(differential_drive_wheel_speeds);
    }

    @Override
    public void periodic() 
    {
        SmartDashboard.putString("GetChassisSpeeds()", GetChassisSpeeds().toString());
        
        // SmartDashboard.putString("Left Velocity", String.format("%n RPM", GetLeftMotorVelocity()));
        // SmartDashboard.putString("Right Velocity", String.format("%n RPM", GetRightMotorVelocity()));
    }
}