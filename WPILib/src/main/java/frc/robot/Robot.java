package frc.robot;

import java.time.Duration;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.GameCommands;
import frc.robot.commands.RampLauncherCommand;

public class Robot extends TimedRobot 
{	
	private final SendableChooser<String> m_chooser = new SendableChooser<String>();

	@Override
	public void simulationInit() 
	{
		System.out.println("Robot is running in simulation!");
	}

	@Override
	public void robotInit() 
	{
		System.out.println("Robot has initialized!");	

		CommandScheduler.getInstance().registerSubsystem(RobotContainer.Drive);
		CommandScheduler.getInstance().registerSubsystem(RobotContainer.Launcher);
		CommandScheduler.getInstance().registerSubsystem(RobotContainer.Localization);
		// CommandScheduler.getInstance().registerSubsystem(RobotContainer.Arm);

		SmartDashboard.putData(Constants.Field);
		// SmartDashboard.putData(RobotContainer.Arm);
		SmartDashboard.putData(RobotContainer.Drive);
		SmartDashboard.putData(RobotContainer.Launcher);
		SmartDashboard.putData(RobotContainer.Localization);
		
		AutoBuilder.configureRamsete(
			RobotContainer.Localization::GetEstimatedPose,
			(pose) -> RobotContainer.Localization.ResetPosition(pose),
			() -> RobotContainer.Drive.GetSpeeds(),
			(targetSpeeds) -> RobotContainer.Drive.Set(targetSpeeds),
			new ReplanningConfig(true, false),
			() -> DriverStation.getAlliance().get() != DriverStation.Alliance.Red, 
			RobotContainer.Drive);
			
		NamedCommands.registerCommand("GotoSpeakerAndLaunch", GameCommands.GotoSpeakerAndLaunch());
		NamedCommands.registerCommand("AutoRotateAndLaunch", GameCommands.AutoRotateAndLaunch());
		NamedCommands.registerCommand("RampLauncher", new RampLauncherCommand(Duration.ofSeconds(1), 1));

		PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
			// Do whatever you want with the pose here
			Constants.Field.setRobotPose(pose);
		});

		// Logging callback for target robot pose
		PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
			// Do whatever you want with the pose here
			Constants.Field.getObject("robot path pose").setPose(pose);
		});

		// Logging callback for the active path, this is sent as a list of poses
		PathPlannerLogging.setLogActivePathCallback((poses) -> {
			// Do whatever you want with the poses here
			Constants.Field.getObject("path").setPoses(poses);
		});
	}

	@Override
	public void robotPeriodic() 
	{
		// Emergency stop on Driver Controller.
		if (Controllers.DriverController.getPOV() == 180 || Controllers.ShooterController.getPOV() == 180) 
		{
			this.EmergencyStop();
			return;
		}

		// Execute / iterate all subsystems, then commands.
		CommandScheduler.getInstance().run();
	}

	public void EmergencyStop()
	{
		CommandScheduler.getInstance().cancelAll();

		RobotContainer.Drive.Stop();
	}

	@Override
	public void teleopInit() 
	{
	}

	@Override
	public void teleopPeriodic() 
	{
		// double forward = Controllers.ApplyDeadzone(Controllers.DriverController.getLeftY());

		// RobotContainer.Drive.LeftLeadMotor.set(forward);
		// RobotContainer.Drive.RightLeadMotor.set(forward);
	}

	@Override
	public void teleopExit() 
	{

	}

	@Override
	public void autonomousPeriodic() 
	{
		var selectedAuto = this.m_chooser.getSelected();
		
		if (selectedAuto == null) return;
	}

	@Override
	public void testInit() 
	{
		var sysId = new SysIdRoutine(new SysIdRoutine.Config(

			Units.Volts.of(0.25).per(Units.Seconds.of(1)),
			Units.Volts.of(0.5),
			Units.Seconds.of(3.6)

		), new SysIdRoutine.Mechanism(
			(voltage) -> 
			{
				System.out.println(voltage);

				// Apply voltages to motors.
				RobotContainer.Drive.Set(Optional.of(new DifferentialDriveWheelVoltages(-voltage.magnitude(), -voltage.magnitude())));
			},
			(log) ->
			{
				log.motor("flywheel")
					.voltage(Units.Volts.of(RobotContainer.Drive.LeftLeadMotor.getBusVoltage()))
					.linearVelocity(Units.MetersPerSecond.of(RobotContainer.Drive.LeftLeadMotor.getEncoder().getVelocity() / 60))
					.linearPosition(Units.Meters.of(RobotContainer.Drive.LeftLeadMotor.getEncoder().getPosition()));
			},
			RobotContainer.Drive));

		sysId
			.quasistatic(Direction.kForward)
			.andThen(Commands.runOnce(() -> System.out.println("Going Back!")))
			.andThen(sysId.quasistatic(Direction.kReverse))
			.andThen(Commands.runOnce(() -> System.out.println("Going Forward!")))
			.andThen(sysId.dynamic(Direction.kForward))
			.andThen(Commands.runOnce(() -> System.out.println("Going Back!")))
			.andThen(sysId.dynamic(Direction.kReverse))
			.finallyDo(() -> RobotContainer.Drive.Stop())
			.schedule();
	}

	@Override
	public void testExit() 
	{
		CommandScheduler.getInstance().cancelAll();
	}
}
