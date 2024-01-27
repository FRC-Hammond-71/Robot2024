package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
// import frc.robot.commands.MoveArmCommand;

public class ArmSubsystem extends SubsystemBase {

    // ------
    // Motors
    // ------ 
    public CANSparkMax ArmMotor = new CANSparkMax(6,MotorType.kBrushless);
    /**
     * Intake & Launcher Motor
     */
    public CANSparkMax IntakeMotor = new CANSparkMax(7, MotorType.kBrushless);

    // private DigitalInput ArmLimitSwitch = new DigitalInput(0);
        
    public XboxController ArmOperatorController = new XboxController(Constants.Controllers.ArmOperatorPort);

    // Arm rests at 103 degrees

    public ArmSubsystem()
    {
        super();

        // this.ArmMotor.setSoftLimit(SoftLimitDirection.kForward, 180);
        this.ArmMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
        // this.ArmMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
        this.ArmMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
        this.ArmMotor.setInverted(false);

        setDefaultCommand(Commands.run(() -> {

            if (ArmOperatorController.getAButton()) {
                this.IntakeMotor.set(-0.85);
            }
            else if (ArmOperatorController.getBButton()) {
                this.IntakeMotor.set(1);
            }
            else if (ArmOperatorController.getYButton()) {
                this.IntakeMotor.set(-0.35);
            }
            else if (ArmOperatorController.getXButton()) {
                this.IntakeMotor.set(0.2);
            } else {
                this.IntakeMotor.stopMotor();
            }

            if (ArmOperatorController.getLeftY() > -Constants.Controllers.Deadzone && ArmOperatorController.getLeftY() < Constants.Controllers.Deadzone)
            {
                // No input from the operator - keep the arm in same position.  
                this.ArmMotor.stopMotor(); 
            }
            else 
            {
                this.ArmMotor.set(ArmOperatorController.getLeftY() * 0.9);
            }
        }, this));
    }

    public Command Calibrate()
    {
        // Use limit-switch to ensure encoder positions are actual.
        throw new UnsupportedOperationException();

        // return new FunctionalCommand(
        //     // Initialize
        //     () -> this.ArmMotor.enableSoftLimit(SoftLimitDirection.kReverse, false), 
        //     // Execute
        //     () -> this.ArmMotor.set(-0.4), 
        //     // On End
        //     (interrupted) -> 
        //     {
        //         this.ArmMotor.stopMotor();
        //         this.ArmMotor.getEncoder().setPosition(0);
        //         this.ArmMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        //     },
        //     // Is Finished
        //     () -> this.ArmLimitSwitch.get(),
        //     // Depend on this subsystem - user input will be ignored.
        //     this);
    }

    // ------------------
    // Movement Reporting
    // ------------------

    // /**
    //  * @return The current rotation of the arm in degrees.
    //  */
    // public double GetArmRotation()
    // {
    //     return ArmMotor.getEncoder().getPosition() * 0.9115044247787611;
    // }

    // /**
    //  * 
    //  * @param position Angle of arm in degrees.
    //  * @param speed Degrees per second.
    //  * @return
    //  */
    // public Command MoveTo(double position)
    // {
    //     return new MoveArmCommand(this, position);
    // }
}