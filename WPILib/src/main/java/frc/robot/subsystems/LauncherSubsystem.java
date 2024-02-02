package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Controllers;

public class LauncherSubsystem extends SubsystemBase {
    // https://www.revrobotics.com/rev-11-1271/
    private DutyCycleEncoder PitchEncoder;

    // https://www.revrobotics.com/rev-21-1650/
    private CANSparkMax PitchMotor;

    // 1. Know the weight of the arm (W)
    // 19.277 lb.

    // 2. Estimate the center of mass of the turret.
    // (X, Y, Z) (-7.354 in, 4.203 in, 2.673 in) Facing from the Back (Intake Closest)

    // 3. Figure out the position of the Joint
    // (X, Y, Z) (0.375 in, 0 in, -0.289 in)

    // 3. Calculate S, the distance from the join to the center of mass of the turret.
    // 4.203 inches  

    // τg = Torque on join applied by gravity.     
    // tg = W * S * cos(0)

    // https://www.chiefdelphi.com/t/neo-motors-pulling-only-5-amps-when-stalled/350542
    // Resistance may be 0.0365 Ohms, unsure.

    // Kv = 473
    // KT is determined by Stall Torque (4.69 N-m) / (Stall Current (257 amps) - Free Current (1.5 amps)).
    // NEO 1.1 Kt = 2.6 Nm / (105 A - 1.8 A) = 0.0251937984496

    // V = ((W * S * R) / Kt) * cos(0)
    // V = ((19.277 lb * 4.203 * 0.0365) / 0.0251937984496 * 20 (GEAR RATION 20:1)) * cos0

    // https://www.revrobotics.com/rev-21-1651/
    private CANSparkMax IntakeMotor;
    // https://www.revrobotics.com/rev-21-1650/
    private CANSparkMax GroundIntakeMotor;
    // https://www.revrobotics.com/rev-21-1650/
    private CANSparkMax LaunchMotor;

    private DigitalInput RotationLimitSwitch;

    private ArmFeedforward PitchFeedForward = new ArmFeedforward(0, 0, 0);
    // private PIDController LaunchMotorPID = new PIDController(0, 0, 0);

    public boolean PitchLimitsEnabled = true;

    public LauncherSubsystem() 
    {
        if (RobotBase.isReal()) 
        {
            this.PitchEncoder = new DutyCycleEncoder(Constants.Launcher.PitchEncoderChannel);

            this.PitchMotor = new CANSparkMax(Constants.Launcher.PitchMotorCANPort, MotorType.kBrushless);
            this.IntakeMotor = new CANSparkMax(Constants.Launcher.IntakeMotor.CANPort, MotorType.kBrushless);
            this.LaunchMotor = new CANSparkMax(Constants.Launcher.LaunchMotor.CANPort, MotorType.kBrushless);
            this.GroundIntakeMotor = new CANSparkMax(Constants.GroundIntake.Motor.Port, MotorType.kBrushless);

            this.RotationLimitSwitch = new DigitalInput(Constants.Launcher.RotationLimitSwitchChannel);
        } else {
            // TODO: Support simulation of Robot
            throw new UnsupportedOperationException("GroundIntakeSubsystem cannot be simulated!");
        }

        // For debugging
        setDefaultCommand(Commands.run(() -> 
        {
            this.IntakeMotor.set(Controllers.DriverController.getAButton() ? 1 : 0);
        }));
    }

    
    public Rotation2d GetPitch() 
    {
        return Rotation2d.fromDegrees(this.PitchEncoder.get() * 360);
    }
    
    /**
     * @return Whether or not the launcher is in the correct position to be loaded with a note.
     */
    public boolean CanBeLoaded()
    {
        return Math.abs(this.GetPitch().minus(Constants.Launcher.LoadingPitch).getDegrees()) < Constants.Launcher.MaximumLoadingPitchError.getDegrees();
    }

    /**
     * @return Whether or not the launcher is loaded with a note.
     */
    public boolean IsLoaded() 
    {
        throw new UnsupportedOperationException("Note detection is not setup on the Launcher!");
    }

    public Command RunGroundIntake()
    {
        return Commands.runEnd(() -> this.GroundIntakeMotor.set(1), () -> this.GroundIntakeMotor.stopMotor(), this);
    }

    // https://www.chiefdelphi.com/t/understanding-feedforward-v-feedback-and-how-their-calculated/186889/10
    public Command Pitch(Rotation2d targetPitch)
    {
        return new FunctionalCommand(
            // Initalize
            () -> {}, 
            // Execute 
            () ->
            {
                var pitchError = targetPitch.minus(this.GetPitch());

                // this.PitchMotor.
            },
            // On End
            (interrupted) -> this.PitchMotor.stopMotor(), 
            // Is Finished
            this::IsLoaded,
            // Requirements 
            this);            
    }

    public Command Calibrate() 
    {
        return new FunctionalCommand(
                // Initialize
                () -> this.PitchLimitsEnabled = false,
                // Execute
                () -> this.PitchMotor.set(-0.1),
                // On End
                (interrupted) -> {
                    this.PitchMotor.stopMotor();
                    this.PitchEncoder.reset();
                    this.PitchLimitsEnabled = true;
                },
                // Is Finished
                () -> this.RotationLimitSwitch.get(),
                this);
    }

    @Override
    public void periodic() 
    {
        // Java doesn't have operator overloading!? :annoyed:
        // TODO: Allow directions which converge with the limits!
        if (this.PitchLimitsEnabled && this.GetPitch().getDegrees() <= 0
                || this.GetPitch().getDegrees() >= Constants.Launcher.MaxPitch.getDegrees()) {
            this.PitchMotor.stopMotor();
        }
    }
}
