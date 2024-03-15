package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ControllerCommands;
import frc.robot.commands.GameCommands;
import frc.robot.commands.UntilNoteLoadedCommand;
import frc.robot.subsystems.ArmPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LaunchSubsystem;
import frc.robot.subsystems.LocalizationSubsystem;

public class Robot extends TimedRobot  
{		
	public final SendableChooser<Boolean> UseAssistedNoteIntake = new SendableChooser<>();

	public static ArmSubsystem Arm;
	public static DriveSubsystem Drive;
	public static LaunchSubsystem Launcher;
	public static LocalizationSubsystem Localization;
	
	//pick and choose
	public static SendableChooser<Integer> SPosition = new SendableChooser<>();
	public static SendableChooser<Command> AutoOptions = new SendableChooser<>();

	public static SendableChooser<Boolean> AutoSpinUp = new SendableChooser<>();

	public Robot()
	{		
		System.out.println("Waiting for connection to Driver Station...");		
		while (!DriverStation.waitForDsConnection(2))
		{
			System.out.println("Retrying connection to Driver Station...");
		}
		System.out.println("Connected to Driver Station!");

		// Starting position need to be declared before subsystems initialize! 
		SPosition.addOption("Amp", 3);
		SPosition.setDefaultOption("Center", 2);
		SPosition.addOption("Source", 1);
		
		Drive = new DriveSubsystem(this);
		Localization = new LocalizationSubsystem(this);
		Arm = new ArmSubsystem(this);
		Launcher = new LaunchSubsystem(this);
		
		// this.UpdateStartingPosition();
		SPosition.onChange((value) -> this.UpdateStartingPosition());
	}

	public void UpdateStartingPosition()
	{
		var pose = FieldConstants.GetStartingPosition();

		if (pose.isPresent())
		{
			// This is an issue with how our simulation is handled... and the methodology of our subsystems.
			if (RobotBase.isReal()) Localization.ResetPosition(pose.get());
			else Robot.Drive.SimulatedDrive.setPose(pose.get());
			DataLogManager.log("Reset starting position!");
		}
	}

	@Override
	public void robotInit() 
	{		
		LEDs.Setup();

		// These are all the command which are invoked in PathPlanner. Commands are mapped to their name!
		NamedCommands.registerCommand("AutoPitchAndLaunch", GameCommands.AutoPitchAndLaunch());
		NamedCommands.registerCommand("AutoRotateAndLaunch", GameCommands.AutoRotateAndLaunch());
		NamedCommands.registerCommand("UntilNoteLoaded", new UntilNoteLoadedCommand());
		NamedCommands.registerCommand("UntilIntakeAngle", Arm.RunUntilHolding(ArmPosition.Intake));
		NamedCommands.registerCommand("IntakeNote", GameCommands.IntakeNote());
		NamedCommands.registerCommand("BeginTrackingSpeaker", GameCommands.BeginTrackingSpeaker());

		// Configure PathPlanner to use the LTV path follower! (LTV appears to work better than ramsete)
		AutoBuilder.configureLTV(
			Localization::GetEstimatedPose180, 
			(pose) -> Localization.ResetPosition(pose),
			() -> Drive.GetSpeeds(), 
			(speeds) -> Drive.Set(speeds), 
			0.02,
			new ReplanningConfig(true, false), 
			() -> DriverStation.getAlliance().get() != DriverStation.Alliance.Red, 
			Drive);

		PathPlannerLogging.setLogCurrentPoseCallback((pose) -> Constants.Field.setRobotPose(pose));

		// Logging callback for target robot pose
		PathPlannerLogging.setLogTargetPoseCallback((pose) -> Constants.Field.getObject("robot path pose").setPose(pose));

		// Logging callback for the active path, this is sent as a list of poses
		PathPlannerLogging.setLogActivePathCallback((poses) -> Constants.Field.getObject("path").setPoses(poses));

		// Work around for Glass not supporting boolean widgets.
		AutoSpinUp.addOption("Enabled", true);
		AutoSpinUp.setDefaultOption("Disabled", false);

		AutoOptions = new SendableChooser<>();
		AutoOptions.setDefaultOption("None", Commands.none());
		AutoOptions.addOption("Amp-A1-2N", new PathPlannerAuto("Amp-A1-2N"));
		AutoOptions.addOption("Amp-A1-M1-3N", new PathPlannerAuto("Amp-A1-M1-3N"));
		AutoOptions.addOption("Center-1N", new PathPlannerAuto("Center-1N"));
		AutoOptions.addOption("Center-2N", new PathPlannerAuto("Center-2N"));
		AutoOptions.addOption("Center-A2-M1-3N", new PathPlannerAuto("Center-A2-M1-3N"));
		AutoOptions.addOption("Center-A2-M2-3N", new PathPlannerAuto("Center-A2-M2-3N"));
		AutoOptions.addOption("Source-M5-2N", new PathPlannerAuto("Source-M5-2N"));
		AutoOptions.addOption("Source-O-1N", new PathPlannerAuto("Source-O-1N"));

		SmartDashboard.putData(Constants.Field);
		SmartDashboard.putData("Starting Positions", SPosition);
		SmartDashboard.putData("Auto Options", AutoOptions);
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

		LEDs.Rainbow();

		// Execute / iterate all subsystems, then commands.
		CommandScheduler.getInstance().run();
	}

	@Override
	public void teleopInit()
	{
		Drive.setDefaultCommand(Commands.run(() -> 
		{
			final double x = -Controllers.DriverController.getLeftY();
			final double rotation = Controllers.DriverController.getRightX();

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
					.andThen(Launcher.AutoIntake()
					.onlyWhile(() -> Controllers.ShooterController.getYButton())
					.finallyDo(() -> 
					{
						ControllerCommands.RumbleController(Controllers.DriverController, RumbleType.kBothRumble, 4, 0.3).schedule();
						ControllerCommands.RumbleController(Controllers.ShooterController, RumbleType.kBothRumble, 4, 0.3).schedule();
					}))
					.schedule();
			}
			else if (Controllers.ShooterController.getXButton())
			{
				Arm.RunUntilHolding(ArmPosition.BySideSpeaker).andThen(Launcher.RunLaunch(0.7, 0.7)).schedule();
			}
			// Retake note
			else if (Controllers.ShooterController.getAButton())
			{
				speed = -0.1;
				Launcher.IntakeMotor.set(-0.1);
				Launcher.FeederMotor.set(-0.1);
			}
			else if (Controllers.ShooterController.getBButton())
			{
				// SHOOT ACROSS MAP
				Arm.RunUntilHolding(ArmPosition.AcrossMap).andThen(Launcher.RunLaunch(0.8, 0.8)).schedule();
			}
			else
			{
				Launcher.IntakeMotor.stopMotor();
				Launcher.FeederMotor.stopMotor();
			}
			
			Launcher.SetLaunchSpeed(speed);

		}, Launcher).withName("Launcher Teleop Command"));

		Arm.setDefaultCommand(Commands.run(() ->
		{
			if (Controllers.ShooterController.getRightBumperPressed())
			{
				// Arm will begin tracking teh speaker!
				Arm.Mode = ArmPosition.TrackingSpeaker;
			}
			else if (Controllers.ShooterController.getLeftBumperPressed())
			{
				// Arm will rotate to the Amp shooting position!
				Arm.Mode = ArmPosition.Amp;
			}

			// if (Controllers.DriverController.getAButton())
			if (Controllers.ShooterController.getRightTriggerAxis() > 0.3)
			{
				GameCommands.AutoPitchAndLaunch().schedule();
				// Enable with caution - needs to be tested!
				// GameCommands.AutoRotateAndLaunch().schedule();
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