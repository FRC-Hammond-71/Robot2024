package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import frc.robot.LEDs;
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
    
    private ArmFeedforward FeedForward;
    
    private PIDController PositionalPID;

    public ArmPosition Mode = ArmPosition.Default;

    public final ArmVisualization Visualization = new ArmVisualization();
    
    public ArmSubsystem(Robot robot) 
    {
        super(robot);    
        
        this.PositionalPID.setTolerance(1);
        this.PositionalPID.setSetpoint(90);

        SmartDashboard.putData("Arm PID", this.PositionalPID);
    }

    @Override
    protected void initializeReal()
    {
        this.PositionalPID = new PIDController(3, 0, 0.4);
        
        this.FeedForward = new ArmFeedforward(0.1, 0.05, 0.05);
        
        this.Motor = new CANSparkMax(Constants.Arm.PitchMotorCANPort, MotorType.kBrushless);
        this.Motor.setSmartCurrentLimit(25);
        this.Motor.setIdleMode(IdleMode.kBrake);
        this.Motor.setInverted(false);
        
        this.AbsoluteEncoder = new DutyCycleEncoder(new DigitalInput(1));
        this.RelativeEncoder = new Encoder(2, 3);

    }

    @Override
    protected void initializeSimulated()
    {
        this.FeedForward = new ArmFeedforward(0, 0, 0.15);

        this.SimulatedArm = new SingleJointedArmSim(
            DCMotor.getNEO(1),
            Constants.Arm.Gearing, 
            Constants.Arm.MomentOfInertia,
            edu.wpi.first.math.util.Units.inchesToMeters(11), 
            0, 
            Constants.Arm.MaxAngle.getRadians(), 
            false, 
            Math.PI / 2);

        this.PositionalPID = new PIDController(4, 0, 0.4);
    }

    /**
     * @return The angle the Arm is rotating per second.
     */
    public Rotation2d GetAngularVelocity()
    {
        if (RobotBase.isReal())
        {
            System.out.println(this.RelativeEncoder.getRate());
            return Rotation2d.fromRotations(this.RelativeEncoder.getRate());
        }
        else return Rotation2d.fromRadians(this.SimulatedArm.getVelocityRadPerSec());
    }

    public Rotation2d GetAngle() 
    {
        return Rotation2d.fromRadians(RobotBase.isReal() ? this.AbsoluteEncoder.getAbsolutePosition() * 2 * Math.PI + edu.wpi.first.math.util.Units.degreesToRadians(4.8) : this.SimulatedArm.getAngleRads());
    }

    public Rotation2d GetAngleError()
    {
        return this.Mode.GetAngle().minus(this.GetAngle());
    }

    public boolean InBounds()
    {
        return Rotation2dUtils.InBounds(this.GetAngle(), Constants.Arm.MinAngle, Constants.Arm.MaxAngle);
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
        return Math.abs(this.GetAngle().minus(rotation).getDegrees()) <= Constants.Arm.AllowedAngleError.getDegrees();
    }

    public boolean IsHolding()
    {
        return this.PositionalPID.atSetpoint();
    }

    public void Stop()
    {        
        if (RobotBase.isReal()) this.Motor.stopMotor();
        else this.SimulatedArm.setInputVoltage(0);

        this.Mode = ArmPosition.Identity;
    }
    
    protected void UpdateMotors()
    {
        if (DriverStation.isDisabled()) 
        {
            this.Stop();
            return;
        }

        Rotation2d desiredAngle = this.Mode.GetAngle();
        desiredAngle = Rotation2d.fromRadians(
            Math.max(Constants.Arm.MinAngle.getRadians(),
            Math.min(desiredAngle.getRadians(), Constants.Arm.MaxAngle.getRadians())));

        var outputRot = this.PositionalPID.calculate(this.GetAngle().getDegrees(), desiredAngle.getDegrees());
        outputRot = Math.min(outputRot, Constants.Arm.MaxSpeed.getDegrees());

        SmartDashboard.putNumber("Arm PID Rotation", outputRot);

        boolean isPushingMax = this.GetAngle().getDegrees() >= Constants.Arm.MaxAngle.getDegrees() + 1 && outputRot > 0;
        boolean isPushingMin = this.GetAngle().getDegrees() <= Constants.Arm.MinAngle.getDegrees() - 1 && outputRot < 0;
        if (isPushingMin || isPushingMax)
        {
            // If PID is responding in a way which goes beyond the soft-limits. DO NOT ALLOW IT!
            this.Stop();
            return;
        }

        double voltage = this.FeedForward.calculate(this.GetAngle().getRadians(), outputRot);

        if (RobotBase.isReal())
        {
            this.Motor.setVoltage(voltage);
        }
        else
        {
            this.SimulatedArm.setInput(voltage);
        }
    }

    @Override
    public void periodic()
    {
        super.periodic();

        if (!DriverStation.isDisabled()) this.UpdateMotors();

       /*  if (this.IsHolding())
        {
            LEDs.SetArm(146, 70, 53);
        }
        else LEDs.SetArm(0, 53, 47); */

        this.Visualization.Update(this);
    }

    @Override
    public void simulationPeriodic() 
    {
        this.SimulatedArm.update(0.02);
    }

    /**
     * @return A command which once scheduled, rotates the Arm to the desired rotation and ends.
     */
    public Command RunUntilHolding(ArmPosition position)
    {
        return Commands.startEnd(() -> this.Mode = position, () -> {}).onlyWhile(() -> !this.IsHolding());
    }

    @Override
    public void initSendable(SendableBuilder builder) 
    {
        builder.addStringProperty("Mode", () -> this.Mode.name(), null);

        builder.addBooleanProperty("Holding", this::IsHolding, null);

        builder.addDoubleProperty("Angular Velocity", () -> this.GetAngularVelocity().getDegrees(), null);
        
        this.addChild("Visualization", this.Visualization);
    }
}
