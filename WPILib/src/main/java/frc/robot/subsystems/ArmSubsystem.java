package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import java.util.Optional;

import javax.swing.text.html.Option;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.RobotSubsystem;
import frc.robot.Constants;
import frc.robot.Controllers;
import frc.robot.Robot;
import frc.robot.utilities.Rotation2dUtils;

public class ArmSubsystem extends RobotSubsystem<frc.robot.Robot> 
{
    private SingleJointedArmSim SimulatedArm;

    // https://www.revrobotics.com/rev-21-1651/
    public CANSparkMax Motor;
    
    // https://www.revrobotics.com/rev-11-1271/
    public DutyCycleEncoder AbsoluteEncoder;
    public Encoder RelativeEncoder;


    private final ArmFeedforward FeedForward = new ArmFeedforward(3, 0.2, 0.1);
    // public final PIDController PID = new PIDController(Math.PI / 2, Math.PI / 2, Math.PI / 4);
    // public final PIDController PID = new PIDController(0.8, 0.2, 0.4);

    private Rotation2d TargetRotation = Rotation2d.fromDegrees(90);

    public final ArmVisualization Visualization = new ArmVisualization();
    
    private Optional<Double> OverrideRotation = Optional.empty() ;

    public ArmSubsystem(Robot robot) 
    {
        super(robot);        

        // this.PID.setTolerance(Constants.Arm.AllowedAngleError.getRadians());
        // this.PID.setSetpoint(Rotation2d.fromDegrees(90).getRadians());
    }

    @Override
    protected void initializeReal()
    {
        this.Motor = new CANSparkMax(Constants.Arm.PitchMotorCANPort, MotorType.kBrushless);
        this.Motor.setIdleMode(IdleMode.kBrake);
        this.Motor.setInverted(false);

        this.AbsoluteEncoder = new DutyCycleEncoder(new DigitalInput(1));
        this.RelativeEncoder = new Encoder(2, 3);
    }

    @Override
    protected void initializeSimulated()
    {
        this.SimulatedArm = new SingleJointedArmSim(
            DCMotor.getNEO(1),
            Constants.Arm.Gearing, 
            Constants.Arm.MomentOfInertia,
            edu.wpi.first.math.util.Units.inchesToMeters(11), 
            0, 
            Constants.Arm.MaxAngle.getRadians(), 
            false, 
            Constants.Arm.MaxAngle.getRadians());
    }

    /**
     * @return Rotations per second.
     */
    public Rotation2d GetVelocity()
    {
        if (RobotBase.isReal())
        {
            return Rotation2d.fromRotations(this.RelativeEncoder.getRate());
        }
        else return Rotation2d.fromRadians(this.SimulatedArm.getAngleRads());
    }

    public Rotation2d GetActualAngle() 
    {
        return Rotation2d.fromRadians(RobotBase.isReal() ? this.AbsoluteEncoder.getAbsolutePosition() * 2 * Math.PI -edu.wpi.first.math.util.Units.degreesToRadians(54) : this.SimulatedArm.getAngleRads());
    }

    public Rotation2d GetTargetAngle()
    {
        return this.TargetRotation;
    }

    public Rotation2d GetAngleError()
    {
        return this.GetTargetAngle().minus(this.GetActualAngle());
    }

    public boolean InBounds()
    {
        return Rotation2dUtils.InBounds(this.GetActualAngle(), Constants.Arm.MinAngle, Constants.Arm.MaxAngle);
    }
    public boolean InBounds(Rotation2d rotation)
    {
        return Rotation2dUtils.InBounds(rotation, Constants.Arm.MinAngle, Constants.Arm.MaxAngle);
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
        return this.IsAt(this.TargetRotation);
    }

    // /**
    //  * @return Whether or not the arm is in the correct position to be loaded with a note.
    //  */
    // public boolean CanBeLoaded()
    // {
    //     return this.IsAt(Constants.Arm.LoadingAngle);
    // }

    public void Stop()
    {
        if (RobotBase.isReal()) this.Motor.stopMotor();
        else this.SimulatedArm.setInputVoltage(0);

        this.OverrideRotation = Optional.empty();
        this.TargetRotation = this.GetActualAngle();

        // this.PID.setSetpoint(this.GetActualAngle().getRadians());
    }

    public void SetAngle(Rotation2d rotation)
    {
        rotation = Rotation2d.fromRadians(Math.max(
            Constants.Arm.MinAngle.getRadians(), Math.min(rotation.getRadians(), Constants.Arm.MaxAngle.getRadians())));

        this.TargetRotation = rotation;
        // this.PID.setSetpoint(rotation.getRadians());
        
        // System.out.printf("Set rotation: %.2f\n", rotation.getDegrees());

        this.UpdateMotors();
    }
    private void SetAngle(Optional<Double> voltage)
    {
        this.OverrideRotation = voltage;
        this.UpdateMotors();
    }


    protected void UpdateMotors()
    {
        // https://www.chiefdelphi.com/t/understanding-feedforward-v-feedback-and-how-their-calculated/186889/10
        if (this.OverrideRotation.isPresent())
        {
            if (RobotBase.isReal()) this.Motor.setVoltage(this.OverrideRotation.get());
            else this.SimulatedArm.setInputVoltage(this.OverrideRotation.get());
            
            return;
        }

        // if (!this.InBounds())
        // {
        //     this.Stop();
        //     this.TargetRotation = Rotation2d.fromDegrees(90);
        //     return;
        // }
        
        if (this.IsHolding())
        {
            if (RobotBase.isReal())
            {
                this.Motor.setVoltage(this.FeedForward.calculate(this.GetActualAngle().getRadians(), 0));
            }
            else
            {
                this.SimulatedArm.setInput(this.FeedForward.calculate(this.GetActualAngle().getRadians(), 0));
            }
        }
        else 
        {
            // var vel = this.PID.calculate(this.GetActualAngle().getRadians());

            if (RobotBase.isReal())
            {
                // System.out.println(this.FeedForward.calculate(this.GetActualAngle().getRadians(), this.GetAngleError().getRadians() * 2));
                this.Motor.setVoltage(this.FeedForward.calculate(this.GetActualAngle().getRadians(), this.GetAngleError().getRadians() * 0.5));
            }
            else
            {
                this.SimulatedArm.setInput(this.FeedForward.calculate(this.GetActualAngle().getRadians(), this.GetAngleError().getRadians() * 1));
            }
        }
    }

    @Override
    public void periodic()
    {
        super.periodic();

        // if (!DriverStation.isDisabled()) this.UpdateMotors();
        this.UpdateMotors();

        this.Visualization.Update(this);
    }

    @Override
    public void simulationPeriodic() 
    {
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
        
        builder.addDoubleProperty("Velocity", () -> this.GetVelocity().getDegrees(), null);

        builder.addDoubleProperty("Voltage", () -> RobotBase.isReal() ? this.Motor.getBusVoltage() : this.SimulatedArm.getOutput(0), null);
        
        this.addChild("Visualization", this.Visualization);
    }

    /**
     * @return A command which once scheduled, rotates the Arm to the desired rotation and ends.
     */
    public Command RunRotate(Rotation2d rotation)
    {
        return Commands
            .startEnd(() -> this.SetAngle(rotation), () -> {}, this)
            .onlyWhile(() -> !this.IsAt(rotation));
    }

    public Command PerformSysID()
    {
        System.out.println("Starting Arm SysID...");

        // Give 5 degrees of headroom!
        Rotation2d upperBound = Constants.Arm.MaxAngle.minus(Rotation2d.fromDegrees(5));
        Rotation2d lowerBound = Constants.Arm.MinAngle.plus(Rotation2d.fromDegrees(8));

        var sysId = new SysIdRoutine(new SysIdRoutine.Config(
			edu.wpi.first.units.Units.Volts.of(0.5).per(edu.wpi.first.units.Units.Seconds.of(1)),
			edu.wpi.first.units.Units.Volts.of(1),
			edu.wpi.first.units.Units.Seconds.of(5.25)), 
            
            new SysIdRoutine.Mechanism(
                (voltage) -> this.SetAngle(Optional.of(voltage.magnitude())),
			    // (log) -> log.motor("arm")
                //             .voltage(edu.wpi.first.units.Units.Volts.of(this.Motor.getBusVoltage()))
                //             .angularVelocity(edu.wpi.first.units.Units.Radians.of(this.GetVelocity().getRadians()).per(edu.wpi.first.units.Units.Minutes.of(1)))
                //             .angularPosition(edu.wpi.first.units.Units.Radians.of(this.GetActualAngle().getRadians())),
                (log) -> {},
        this));

		return this.RunRotate(lowerBound) 
            .andThen(new WaitCommand(1))
            .andThen(sysId.quasistatic(Direction.kForward)
            .andThen(new WaitCommand(1))
			.andThen(sysId.quasistatic(Direction.kReverse))
            .andThen(new WaitCommand(1))
			.andThen(sysId.dynamic(Direction.kForward))
            .andThen(new WaitCommand(1))
			.andThen(sysId.dynamic(Direction.kReverse))
            .onlyWhile(() -> Rotation2dUtils.InBounds(this.GetActualAngle(), lowerBound, upperBound))
			.finallyDo((interrupted) ->
            {
                this.Stop();

                if (interrupted)
                {
                    System.out.println("SysID was interrupted and forcibly ended!");
                }
                else
                {
                    System.out.println("Arm SysID Completed!");
                }
                this.SetAngle(Rotation2d.fromDegrees(90));
            }));
    }
}
