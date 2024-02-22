package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
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
    public CANSparkMax Motor;
    
    // https://www.revrobotics.com/rev-11-1271/
    private DutyCycleEncoder Encoder;

    private final ArmFeedforward PitchFeedForward = new ArmFeedforward(0.5, 0.15, 0.1);
    public final PIDController ArmPID = new PIDController(Math.PI / 2, Math.PI / 2, Math.PI / 4);

    public final ArmVisualization Visualization = new ArmVisualization();

    public ArmSubsystem(Robot robot) 
    {
        super(robot);        

        this.ArmPID.setTolerance(Constants.Arm.AllowedAngleError.getRadians());
    }

    @Override
    protected void initializeReal()
    {
        this.Motor = new CANSparkMax(Constants.Arm.PitchMotorCANPort, MotorType.kBrushless);

        this.Encoder = new DutyCycleEncoder(new DigitalInput(0));

        this.Motor.setIdleMode(IdleMode.kBrake);

        this.Encoder.reset();
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
            false, 
            Constants.Arm.MaxAngle.getRadians());
    }

    public Rotation2d GetActualAngle() 
    {
        return Rotation2d.fromRadians(RobotBase.isReal() ? this.Encoder.get() * 2 * Math.PI : this.SimulatedArm.getAngleRads());
    }

    public Rotation2d GetTargetAngle()
    {
        return Rotation2d.fromRadians(this.ArmPID.getSetpoint());
        // return this.TargetAngle;
    }

    public Rotation2d GetAngleError()
    {
        return this.GetTargetAngle().minus(this.GetActualAngle());
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
        return this.IsAt(Rotation2d.fromRadians(ArmPID.getSetpoint()));
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
        if (RobotBase.isReal()) this.Motor.stopMotor();
        else this.SimulatedArm.setInputVoltage(0);

        // this.TargetAngle = this.GetActualAngle();
        this.ArmPID.setSetpoint(this.GetActualAngle().getRadians());
    }

    public void SetAngle(Rotation2d rotation)
    {
        // this.TargetAngle = rotation;
        this.ArmPID.setSetpoint(rotation.getRadians());
        this.UpdateMotors();
    }

    protected void UpdateMotors()
    {
        // https://www.chiefdelphi.com/t/understanding-feedforward-v-feedback-and-how-their-calculated/186889/10

        if (this.IsHolding())
        {
            if (RobotBase.isReal())
            {
                this.Motor.setVoltage(this.PitchFeedForward.calculate(this.GetActualAngle().getRadians(), 0));
            }
            else
            {
                this.SimulatedArm.setInput(this.PitchFeedForward.calculate(this.GetActualAngle().getRadians(), 0));
            }
        }
        else 
        {
            var vel = this.ArmPID.calculate(this.GetActualAngle().getRadians());

            if (RobotBase.isReal())
            {
                this.Motor.setVoltage(this.PitchFeedForward.calculate(this.GetActualAngle().getRadians(), vel));
            }
            else
            {
                this.SimulatedArm.setInput(this.PitchFeedForward.calculate(this.GetActualAngle().getRadians(), vel * 60));
            }
        }
    }

    @Override
    public void periodic()
    {
        super.periodic();

        this.Visualization.Update(this);
    }

    @Override
    protected void realPeriodic()
    {
        System.out.println(this.Encoder.getAbsolutePosition());
        // this.UpdateMotors();
    }

    @Override
    public void simulationPeriodic() 
    {
        this.UpdateMotors();

        this.SimulatedArm.update(0.02);
    }

    @Override
    public void initSendable(SendableBuilder builder) 
    {
        builder.addDoubleProperty("Target Angle", 
            () -> this.GetTargetAngle().getDegrees(), 
            (deg) -> this.SetAngle(Rotation2d.fromDegrees(deg)));

        builder.addDoubleProperty("Angle", () -> this.GetActualAngle().getDegrees(), null);

        builder.addBooleanProperty("Holding", this::IsHolding, null);

        this.addChild("Visualization", this.Visualization);
        this.addChild("PID", this.ArmPID);
    }

    /**
     * @return A command which once scheduled, rotates the Arm to the desired rotation and ends.
     */
    public Command RunRotate(Rotation2d rotation)
    {
        return Commands
            .startEnd(() -> this.SetAngle(rotation), () -> {}, this)
            .onlyWhile(() -> !this.IsHolding());
    }
}
