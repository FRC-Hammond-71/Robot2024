package frc.robot;

import java.sql.DriverManager;
import java.time.Duration;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.commands.GameCommands;
import frc.robot.commands.PathCommands;
import frc.robot.commands.RampLauncherCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FieldLocalizationSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class Robot extends TimedRobot 
{	
	public static ArmSubsystem Arm;
	public static DriveSubsystem Drive;
	public static LauncherSubsystem Launcher;
	public static FieldLocalizationSubsystem FieldLocalization;

	private final SendableChooser<String> m_chooser = new SendableChooser<String>();

	@Override
	public void simulationInit() 
	{
		System.out.println("Robot is running in simulation!");
	}

	@Override
	public void robotInit() 
	{
		Arm = new ArmSubsystem();
		Drive = new DriveSubsystem();
		Launcher = new LauncherSubsystem();
		FieldLocalization = new FieldLocalizationSubsystem(Drive);

		addPeriodic(() ->
		{
			
		}, kDefaultPeriod);

		// NamedCommands.registerCommand("IntakeNoteAndLoadIntoLauncher", GameCommands.IntakeNoteAndLoadIntoLauncher(Arm, Launcher));

		// NAMED COMMANDS MUST BE REGISTERED BEFORE A PATH AUTO BUILDER IS CONFIGURED!
		
		AutoBuilder.configureRamsete(
			FieldLocalization::GetEstimatedPose,
			(pose) -> FieldLocalization.ResetPosition(pose),
			() -> Drive.GetSpeeds(),
			(targetSpeeds) -> Drive.Set(targetSpeeds),
			new ReplanningConfig(true, false),
			() -> DriverStation.getAlliance().get() != DriverStation.Alliance.Red, 
			Drive);
			
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

		SmartDashboard.putData("Note 1", AutoBuilder.followPath(PathPlannerPath.fromPathFile("Note 1")).andThen(GameCommands.AutoRotateAndLaunch()));

		SmartDashboard.putData("Speaker Auto", Commands.none().andThen(() -> GameCommands.AutoRotateAndLaunch()).withName("Speaker Auto"));

		SmartDashboard.putData(Constants.Field);
		SmartDashboard.putData(Arm);
		SmartDashboard.putData(Drive);
		SmartDashboard.putData(Launcher);
		SmartDashboard.putData(FieldLocalization);

		// CameraServer.startAutomaticCapture();

		Shuffleboard.getTab("Movement").add(Drive);
	}

	@Override
	public void robotPeriodic() 
	{
		// Emergency stop on Driver Controller.
		if (Controllers.DriverController.getPOV() == 180 || Controllers.ShooterController.getPOV() == 180) 
		{
			this.EmergencyStop();
		}

		CommandScheduler.getInstance().run();
	}

	public void EmergencyStop()
	{
		CommandScheduler.getInstance().cancelAll();

		Drive.Stop();
	}

	@Override
	public void teleopInit() 
	{
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
}
