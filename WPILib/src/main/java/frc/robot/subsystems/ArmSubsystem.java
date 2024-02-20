package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.RobotSubsystem;
import frc.robot.Constants;
import frc.robot.Controllers;
import frc.robot.Robot;

public class ArmSubsystem extends RobotSubsystem<frc.robot.Robot> 
{
    private SingleJointedArmSim SimulatedArm;

    // https://www.revrobotics.com/rev-21-1651/
    private CANSparkMax Motor;
    
    // https://www.revrobotics.com/rev-11-1271/
    private DutyCycleEncoder Encoder;

    private DigitalInput LimitSwitch;

    private ArmFeedforward PitchFeedForward = new ArmFeedforward(0.1, 0.2833340973, 0.1);

    public boolean PitchLimitsEnabled = true;
    private Rotation2d TargetAngle = Constants.Arm.MaxAngle;

    public ArmSubsystem(Robot robot) 
    {
        super(robot);
    }

    @Override
    protected void initializeReal()
    {
        this.Motor = new CANSparkMax(Constants.Arm.PitchMotorCANPort, MotorType.kBrushless);

        this.Encoder = new DutyCycleEncoder(Constants.Arm.PitchEncoderChannel);
        this.LimitSwitch = new DigitalInput(Constants.Arm.RotationLimitSwitchChannel);
    }

    @Override
    protected void initializeSimulated()
    {
        this.SimulatedArm = new SingleJointedArmSim(
            DCMotor.getNEO(1),
            Constants.Arm.Gearing, 
            Constants.Arm.MomentOfInertia, 
            Units.inchesToMeters(11), 
            0, 
            Constants.Arm.MaxAngle.getRadians(), 
            true, 
            Constants.Arm.MaxAngle.getRadians());
    }

    public Rotation2d GetActualAngle() 
    {
        return Rotation2d.fromRadians(RobotBase.isReal() ? this.Encoder.get() * 2 * Math.PI : this.SimulatedArm.getAngleRads());
    }

    public Rotation2d GetTargetAngle()
    {
        return this.TargetAngle;
    }

    public Rotation2d GetAngleError()
    {
        return this.TargetAngle.minus(this.GetActualAngle());
    }

    /**
     * @param rotation The desired rotation.
     * @return Whether or not the launcher is in the desired pitch and within Constants.Arm.AllowedAngleError error. 
     */
    public boolean IsAt(Rotation2d rotation)
    {
        return Math.abs(this.GetActualAngle().minus(rotation).getDegrees()) <= Constants.Arm.AllowedAngleError.getDegrees();
    }

    public boolean IsHolding()
    {
        return this.IsAt(this.TargetAngle);
    }

    /**
     * @return Whether or not the arm is in the correct position to be loaded with a note.
     */
    public boolean CanBeLoaded()
    {
        return this.IsAt(Constants.Arm.LoadingAngle);
    }

    public void Stop()
    {
        // if (RobotBase.isReal()) this.Motor.stopMotor();
        // else this.SimulatedArm.setInputVoltage(0);

        this.TargetAngle = this.GetActualAngle();
    }

    public void SetAngle(Rotation2d rotation)
    {
        this.TargetAngle = rotation;
        this.UpdateMotors();
    }

    protected void UpdateMotors()
    {
        // https://www.chiefdelphi.com/t/understanding-feedforward-v-feedback-and-how-their-calculated/186889/10

        var error = this.GetAngleError();

        if (Math.abs(error.getDegrees()) > Constants.Arm.AllowedAngleError.getDegrees())
        {
            this.Motor.setVoltage(this.PitchFeedForward.calculate(this.GetActualAngle().getRadians(), error.getRadians() * 0.25));
        }
        else 
        {
            this.Motor.setVoltage(this.PitchFeedForward.calculate(this.GetActualAngle().getRadians(), 0));
        }
    }

    @Override
    protected void realPeriodic()
    {
        this.UpdateMotors();
    }

    @Override
    public void simulationPeriodic() 
    {
        // this.UpdateMotors();
        this.SimulatedArm.update(0.02);
    }

    @Override
    public void initSendable(SendableBuilder builder) 
    {
        builder.addDoubleProperty("Target Angle", 
            () -> this.TargetAngle.getDegrees(), 
            (deg) -> this.SetAngle(Rotation2d.fromDegrees(deg)));

        builder.addDoubleProperty("Actual Angle", () -> this.GetActualAngle().getDegrees(), null);

        builder.addBooleanProperty("Holding", this::IsHolding, null);
        
        builder.addDoubleProperty("Error", 
            () -> Constants.Arm.AllowedAngleError.getDegrees(), 
            (deg) -> Constants.Arm.AllowedAngleError = Rotation2d.fromDegrees(deg));

    }

    public Command RunCalibration() 
    {
        return new FunctionalCommand(
            // Initialize
            () -> this.PitchLimitsEnabled = false,
            // Execute
            () -> this.SetAngle(Constants.Arm.MaxAngle),
            // On End
            (interrupted) ->
            {
                this.PitchLimitsEnabled = true;

                // For debugging
                System.out.println("Arm calibration complete!");
            },
            // Is Finished
            () -> this.LimitSwitch.get(),
            this);
    }

    /**
     * @return A command which once scheduled, rotates the Arm to the desired position and completes.
     */
    public Command RunRotate(Rotation2d rotation)
    {
        return Commands.startEnd(() -> this.SetAngle(rotation), () -> {}, this).onlyWhile(() -> !this.IsHolding());
    }
}
