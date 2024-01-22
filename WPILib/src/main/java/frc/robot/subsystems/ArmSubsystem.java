package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.MoveArmCommand;

public class ArmSubsystem extends SubsystemBase {

    // ------
    // Motors
    // ------ 
    public CANSparkMax ArmMotor = new CANSparkMax(6,MotorType.kBrushless);
    /**
     * Intake & Launcher Motor
     */
    public CANSparkMax IntakeMotor = new CANSparkMax(7, MotorType.kBrushless);

    private PIDController ArmPID = new PIDController(1, 0, 0.5);

    // Arm rests at 103 degrees

    public ArmSubsystem()
    {
        super();

        this.ArmMotor.setSoftLimit(SoftLimitDirection.kForward, 130);
        this.ArmMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        this.ArmMotor.setSoftLimit(SoftLimitDirection.kReverse, -5);
        this.ArmMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        this.ArmMotor.setInverted(true);
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