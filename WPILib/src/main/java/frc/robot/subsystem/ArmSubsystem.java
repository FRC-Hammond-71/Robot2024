package frc.robot.subsystem;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.MoveArmCommand;

import java.util.Date;

public class ArmSubsystem extends SubsystemBase {

    // ------
    // Motors
    // ------ 
    public CANSparkMax ArmMotor = new CANSparkMax(6,MotorType.kBrushless);
    /**
     * Intake & Launcher Motor
     */
    public CANSparkMax IntakeMotor = new CANSparkMax(7, MotorType.kBrushless);

    // Arm rests at 103 degrees

    public ArmSubsystem()
    {
        this.ArmMotor.setSoftLimit(SoftLimitDirection.kForward, 130);
        this.ArmMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
        this.ArmMotor.setInverted(true);
        // this.ArmMotor.setSoftLimit(SoftLimitDirection.kForward, 1);
    }

    // ------------------
    // Movement Reporting
    // ------------------

    /**
     * @return The current rotation of the arm in degrees.
     */
    public double GetArmRotation()
    {
        return ArmMotor.getEncoder().getPosition() * 0.9115044247787611;
    }

    /**
     * 
     * @param position Angle of arm in degrees.
     * @param speed Degrees per second.
     * @return
     */
    public Command MoveTo(double position)
    {
        return new MoveArmCommand(this, position);
    }
}