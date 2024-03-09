package frc.robot;

import java.time.Duration;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;

import com.fasterxml.jackson.annotation.JacksonInject.Value;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.util.GeometryUtil;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.proto.Controller;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ControllerCommands;
import frc.robot.commands.GameCommands;
import frc.robot.commands.PathCommands;
import frc.robot.commands.UntilNoteLoadedCommand;
import frc.robot.subsystems.ArmPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.LaunchSubsystem;
import frc.robot.subsystems.LocalizationSubsystem;

public class Robot extends TimedRobot  
{		
	public final SendableChooser<Boolean> UseAssistedNoteIntake = new SendableChooser<>();

	public static ArmSubsystem Arm;
	public static Drive Drive;
	public static LaunchSubsystem Launcher;
	public static LocalizationSubsystem Localization;
	
	//pick and choose
	public static SendableChooser<Integer> SPosition = new SendableChooser<>();
	public static SendableChooser<Command> AutoOptions = new SendableChooser<>();

	public static SendableChooser<Boolean> AutoSpinUp = new SendableChooser<>();

	public static PhotonCamera NoteIntakeCamera = new PhotonCamera("Note Intake");

	public Robot()
	{
		SPosition.onChange((value) ->  
		{
			Pose2d pose;

			try
			{
				pose = FieldConstants.GetStartingPosition();
			}
			catch (Exception ex)
			{	
				System.out.println("Could not update starting point." + ex);
				return;
			}

			Localization.ResetPosition(pose);
		});
		
		System.out.println("Waiting for connection to Driver Station...");		
		while (!DriverStation.waitForDsConnection(2))
		{
			System.out.println("Retrying connection to Driver Station...");
		}
		System.out.println("Connected to Driver Station!");
		
		Arm = new ArmSubsystem(this);
		Drive = new Drive(this);
		Launcher = new LaunchSubsystem(this);
		Localization = new LocalizationSubsystem(this);
	}

	@Override
	public void robotInit() 
	{		
		LEDs.Setup();

		NamedCommands.registerCommand("AutoPitchAndLaunch", GameCommands.AutoPitchAndLaunch());
		NamedCommands.registerCommand("AutoRotateAndLaunch", GameCommands.AutoRotateAndLaunch());
		NamedCommands.registerCommand("UntilNoteLoaded", new UntilNoteLoadedCommand());
		NamedCommands.registerCommand("UntilIntakeAngle", Arm.RunUntilHolding(ArmPosition.Intake));
		NamedCommands.registerCommand("IntakeNote", GameCommands.IntakeNote());
		NamedCommands.registerCommand("BeginTrackingSpeaker", GameCommands.BeginTrackingSpeaker());

		AutoBuilder.configureLTV(
			Localization::GetEstimatedPose180, 
			(pose) -> Localization.ResetPosition(pose),
			() -> Drive.GetSpeeds(), 
			(speeds) -> Drive.Set(speeds), 
			0.02, 
			new ReplanningConfig(true, false), 
			() -> DriverStation.getAlliance().get() != DriverStation.Alliance.Red, 
			Drive);

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

		// Work around for Glass not supporting boolean widgets.
		AutoSpinUp.addOption("Enabled", true);
		AutoSpinUp.setDefaultOption("Disabled", false);

		SPosition.setDefaultOption("Center", 2);
		SPosition.addOption("Source", 1);
		SPosition.addOption("Amp", 3);

		AutoOptions = AutoBuilder.buildAutoChooser();

		SmartDashboard.putData("Auto Options", AutoOptions);
		SmartDashboard.putData("Starting Positions", SPosition);
		SmartDashboard.putData(Constants.Field);
		SmartDashboard.putData(Arm);
		SmartDashboard.putData(Drive);
		SmartDashboard.putData(Launcher);
		SmartDashboard.putData(Localization);
		SmartDashboard.putData(CommandScheduler.getInstance());
		SmartDashboard.putData("Arm Visualization", Arm.Visualization);
	}

	public void Stop()
	{
		CommandScheduler.getInstance().cancelAll();
		Drive.Stop();
		Arm.Stop();
		Launcher.Stop();
	}

	@Override
	public void robotPeriodic() 
	{
		// Emergency stop on Driver Controller.
		if (Controllers.DriverController.getPOV() == 180 || Controllers.ShooterController.getPOV() == 180) 
		{
			this.Stop();
			return;
		}

		if (Controllers.DriverController.getBackButtonPressed())
		{
			var driveCommand = CommandScheduler.getInstance().requiring(Drive);

			if (driveCommand != null && driveCommand != Drive.getDefaultCommand())
			{
				CommandScheduler.getInstance().cancel(driveCommand);
			}
		}

		// LEDs.Rainbow();

		// Execute / iterate all subsystems, then commands.
		CommandScheduler.getInstance().run();
	}

	@Override
	public void teleopInit()
	{
		Drive.setDefaultCommand(Commands.run(() -> 
		{
			final double x = -Controllers.DriverController.getLeftY();
			final double rotation = -Controllers.DriverController.getRightX();

            Drive.SetArcade(x, rotation);

			// boolean runNoteAssistance = Controllers.ShooterController.getYButton()
			// 	&& rotation == 0
			// 	&& !Launcher.IsLoaded()
			// 	&& this.NoteIntakeCamera.getLatestResult().hasTargets();

			// if (runNoteAssistance)
			// {
			// 	var result = this.NoteIntakeCamera.getLatestResult().getBestTarget();
			// }

		}, Drive).withName("Drive Teleop Command"));
		
		Launcher.setDefaultCommand(Commands.run(() -> 
		{	
			if (RobotBase.isSimulation()) return;

			var speed = -Controllers.ApplyDeadzone(Controllers.ShooterController.getLeftY()) * 1;
			speed = Math.copySign(speed * speed, speed);

			// if (AutoSpinUp.getSelected().booleanValue() && speed == 0 && Launcher.IsLoaded() && Arm.Mode == ArmPosition.TrackingSpeaker)
			// {
			// 	speed = 0.4;
			// }
			
			// Intake note 
			if (Controllers.ShooterController.getYButton())
			{
				Arm.RunUntilHolding(ArmPosition.Intake)
					.andThen(Launcher.Intake()
					.onlyWhile(() -> Controllers.ShooterController.getYButton())
					.finallyDo(() -> 
					{
						ControllerCommands.RumbleController(Controllers.DriverController, RumbleType.kBothRumble, 4, 0.3).schedule();
						ControllerCommands.RumbleController(Controllers.ShooterController, RumbleType.kBothRumble, 4, 0.3).schedule();
					}))
					.schedule();
			}
			// Retake note
			else if (Controllers.ShooterController.getAButton())
			{
				speed = -0.1;
				Launcher.GroundIntakeMotor.set(-0.1);
				Launcher.IntakeMotor.set(-0.1);
			}
			else
			{
				Launcher.GroundIntakeMotor.stopMotor();
				Launcher.IntakeMotor.stopMotor();
			}
			
			Launcher.SetLaunchSpeed(speed);

		}, Launcher).withName("Launcher Teleop Command"));

		Arm.setDefaultCommand(Commands.run(() ->
		{
			if (Controllers.ShooterController.getRightBumperPressed())
			{
				Arm.Mode = ArmPosition.TrackingSpeaker;
			}
			else if (Controllers.ShooterController.getLeftBumperPressed())
			{
				Arm.Mode = ArmPosition.Amp;
			}

			// if (Controllers.DriverController.getAButton())
			if (Controllers.ShooterController.getRightTriggerAxis() > 0.3)
			{
				GameCommands.AutoRotateAndLaunch().schedule();
			}
			else if (Controllers.ShooterController.getLeftTriggerAxis() > 0.3)
			{
				GameCommands.ScoreAmp().schedule();
			}
			else
			{
				// Arm.SetAngle(Arm.GetTargetAngle().plus(Rotation2d.fromDegrees(Controllers.SquareInput(-Controllers.ApplyDeadzone(Controllers.ShooterController.getRightY())))));	
			}
		}, Arm).withName("Arm Teleop Command"));
	}
	
	@Override
	public void teleopExit()
	{
		CommandScheduler.getInstance().cancel(Launcher.getDefaultCommand());
		CommandScheduler.getInstance().cancel(Drive.getDefaultCommand());
		CommandScheduler.getInstance().cancel(Arm.getDefaultCommand());
		// removeDefaultCommand() does not unschedule / cancel the command.
		Launcher.removeDefaultCommand();
		Drive.removeDefaultCommand();
		Arm.removeDefaultCommand();
	}

	@Override
	public void disabledInit()
	{
		this.Stop();
	}

	@Override
	public void autonomousInit()
	{
		final Command autoCommand = AutoOptions.getSelected();
		if (autoCommand == null)
		{
			DataLogManager.log("Auto not chosen, ignoring.");
			return;
		}

		autoCommand.schedule();
	}

	@Override
	public void autonomousExit()
	{
		this.Stop();
		// CommandScheduler.getInstance().cancel(AutoCommand);
	}
}
