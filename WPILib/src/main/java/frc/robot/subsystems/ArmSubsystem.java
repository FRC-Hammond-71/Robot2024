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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
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
    public DutyCycleEncoder AbsoluteEncoder;
    public RelativeEncoder relativeEncoder;


    private final ArmFeedforward FeedForward = new ArmFeedforward(0.5, 0.15, 0.1);
    // public final PIDController PID = new PIDController(Math.PI / 2, Math.PI / 2, Math.PI / 4);
    public final PIDController PID = new PIDController(1, 0.1, 0.1);


    public final ArmVisualization Visualization = new ArmVisualization();
    
    private Optional<Double> OverrideRotation = Optional.empty() ;

    public ArmSubsystem(Robot robot) 
    {
        super(robot);        

        this.PID.setTolerance(Constants.Arm.AllowedAngleError.getRadians());
        this.PID.setSetpoint(Rotation2d.fromDegrees(90).getRadians());
    }

    @Override
    protected void initializeReal()
    {
        this.Motor = new CANSparkMax(Constants.Arm.PitchMotorCANPort, MotorType.kBrushless);
        this.Motor.setIdleMode(IdleMode.kBrake);
        this.Motor.setInverted(false);

        this.AbsoluteEncoder = new DutyCycleEncoder(new DigitalInput(1));
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

    public Rotation2d GetActualAngle() 
    {
        return Rotation2d.fromRadians(RobotBase.isReal() ? this.AbsoluteEncoder.getAbsolutePosition() * 2 * Math.PI -edu.wpi.first.math.util.Units.degreesToRadians(54) : this.SimulatedArm.getAngleRads());
    }

    public Rotation2d GetTargetAngle()
    {
        return Rotation2d.fromRadians(this.PID.getSetpoint());
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
        return this.IsAt(Rotation2d.fromRadians(PID.getSetpoint()));
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

        // this.PID.setSetpoint(this.GetActualAngle().getRadians());
    }

    public void SetAngle(Rotation2d rotation)
    {
        rotation = Rotation2d.fromRadians(Math.max(
            Constants.Arm.MinAngle.getRadians(), Math.min(rotation.getRadians(), Constants.Arm.MaxAngle.getRadians())));

        this.PID.setSetpoint(rotation.getRadians());
        
        System.out.printf("Set rotation: %.2f\n", rotation.getDegrees());

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
            var vel = this.PID.calculate(this.GetActualAngle().getRadians());

            if (RobotBase.isReal())
            {
                System.out.println(this.FeedForward.calculate(this.GetActualAngle().getRadians(), vel));
                this.Motor.setVoltage(this.FeedForward.calculate(this.GetActualAngle().getRadians(), vel));
            }
            else
            {
                this.SimulatedArm.setInput(this.FeedForward.calculate(this.GetActualAngle().getRadians(), vel * 60));
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
        if (!DriverStation.isDisabled())
        {
            this.UpdateMotors();
        }
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
        this.addChild("PID", this.PID);
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
        // var sysId = new SysIdRoutine(new SysIdRoutine.Config(

		// 	edu.wpi.first.units.Units.Volts.of(0.25).per(edu.wpi.first.units.Units.Seconds.of(1)),
		// 	edu.wpi.first.units.Units.Volts.of(0.5),
		// 	edu.wpi.first.units.Units.Seconds.of(3.4)

		// ), new SysIdRoutine.Mechanism(
		// 	(voltage) -> 
		// 	{
		// 		System.out.println(voltage);

		// 		// Apply voltages to motors.
		// 		this.Set(Optional.of(Motor.)));
		// 	},
		// 	(log) ->
		// 	{
		// 		log.motor("arm")
                
		// 			.voltage(edu.wpi.first.units.Units.Volts.of(this.Motor.getBusVoltage()))
		// 			.angularVelocity(edu.wpi.first.units.Units.Radians.of(this.relativeEncoder.getVelocity()).per(edu.wpi.first.units.Units.Minutes.of(1)))
        //             .angularPosition(edu.wpi.first.units.Units.Radians.of(this.GetActualAngle().getRadians()));
        //     },  
		// 	this));

		// return sysId
		// 	.quasistatic(Direction.kForward)
		// 	.andThen(Commands.runOnce(() -> System.out.println("Going Back!")))
		// 	.andThen(sysId.quasistatic(Direction.kReverse))
		// 	.andThen(Commands.runOnce(() -> System.out.println("Beginning dynamic test...")))
		// 	.andThen(sysId.dynamic(Direction.kForward))
		// 	.andThen(Commands.runOnce(() -> System.out.println("Going Back!")))
		// 	.andThen(sysId.dynamic(Direction.kReverse))
		// 	.finallyDo(() -> this.Stop());

        return Commands.none();
    }
}
