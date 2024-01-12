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

    public DifferentialDrive Drive = new DifferentialDrive(LeftLeadMotor, RightLeadMotor);

    public DriveSubsystem() { }

    // ------------------
    // Movement Reporting
    // ------------------
    /**
     * @return The Left Motor Speed in Meters per Second.
     */
    public double GetLeftMotorSpeed()
    {
        return this.LeftLeadMotor.getEncoder().getVelocity() / 60 * 0.48;
    }
     /**
     * @return The Right Motor Speed in Meters per Second.
     */
    public double GetRightMotorSpeed()
    {
        return this.RightLeadMotor.getEncoder().getVelocity() / 60 * 0.48;
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
            GetLeftMotorSpeed(),
            GetRightMotorSpeed()
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