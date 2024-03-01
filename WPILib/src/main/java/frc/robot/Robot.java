package frc.robot;

import java.time.Duration;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.GeometryUtil;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.proto.Controller;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.IPeriodic;
import frc.robot.commands.GameCommands;
import frc.robot.commands.PathCommands;
import frc.robot.commands.UntilNoteLoadedCommand;
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

	public Robot()
	{
		System.out.println("Waiting for connection to Driver Station...");
		
		while (!DriverStation.waitForDsConnection(10))
		{
			System.out.println("Retrying connection to Driver Station...");
		}
		System.out.println("Connected to Driver Station!");
		
		Arm = new ArmSubsystem(this);
		Drive = new DriveSubsystem(this);
		Launcher = new LaunchSubsystem(this);
		Localization = new LocalizationSubsystem(this);
	}

	@Override
	public void simulationInit() 
	{
		System.out.println("Robot is running in simulation!");
	}

	@Override
	public void robotInit() 
	{
		System.out.println("Robot has initialized!");
		
		// LEDs.Setup();

		// IPeriodic.ApplyPeriodic(Localization.RelativeSensorUpdatePeriodic, this);
		// IPeriodic.ApplyPeriodic(Localization.VisionUpdatePeriodic, this); 

		SmartDashboard.putData(Constants.Field);
		SmartDashboard.putData(Arm);
		SmartDashboard.putData(Drive);
		SmartDashboard.putData(Launcher);
		SmartDashboard.putData(Localization);

		SmartDashboard.putData("Arm Visualization", Arm.Visualization);
		// SmartDashboard.putData("Assisted Note Intake", this.UseAssistedNoteIntake);
		// SmartDashboard.putData("Arm PID", Arm.PID);
		
		AutoBuilder.configureRamsete(
			Localization::GetEstimatedPose,
			(pose) -> Localization.ResetPosition(pose),
			() -> Drive.GetSpeeds(),
			(targetSpeeds) ->
			{
				System.out.println(targetSpeeds.toString());
				Drive.Set(targetSpeeds);
			},
			new ReplanningConfig(true, false),
			() -> DriverStation.getAlliance().get() != DriverStation.Alliance.Red, 
			Drive);
			
		// NamedCommands.registerCommand("GotoSpeakerAndLaunch", GameCommands.GotoSpeakerAndLaunch());
		NamedCommands.registerCommand("AutoRotateAndLaunch", GameCommands.AutoRotateAndLaunch());
		// NamedCommands.registerCommand("RampLauncher", new RampLauncherCommand(Duration.ofSeconds(1), 1));
		NamedCommands.registerCommand("UntilNoteLoaded", new UntilNoteLoadedCommand());
		NamedCommands.registerCommand("IntakeNote", GameCommands.IntakeNote());
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
			// if (Controllers.DriverController.getLeftBumperPressed() && RobotBase.isSimulation())
			// {
			// 	Localization.ResetPosition(new Pose2d(16.54 / 2, 5, new Rotation2d()));
			// }
 
            // if (Controllers.DriverController.getAButtonPressed())
            // {
            //     System.out.println("Path finding to Speaker!");
            //     PathCommands.PathToSpeaker().schedule();
            // }
            // if (Controllers.DriverController.getBButtonPressed())
            // {
            //     System.out.println("Path finding to Amp!");
            //     PathCommands.PathToAmplifier().schedule();
            // }

            Drive.SetArcade(-Controllers.DriverController.getLeftY(), -Controllers.DriverController.getRightX());

		}, Drive));
		
		Launcher.setDefaultCommand(Commands.run(() -> 
		{
			// if (Controllers.DriverController.getRightBumperPressed())
			// {
			// 	GameCommands.AutoRotateAndLaunch().schedule();
			// 	return;
			// }

			if (Controllers.ShooterController.getLeftBumperPressed())
			{
				Arm.RunRotate(Rotation2d.fromDegrees(100)).andThen(Launcher.Launch(0.28, 0.10)).schedule();
				return;
			}

			if (RobotBase.isSimulation()) return;

			// Reset the logic which determines if a note is loaded if something goes wrong.
			if (Controllers.ShooterController.getPOV() == 180)
			{
				Launcher.SetLoaded(false);
			}
			else if (Controllers.ShooterController.getPOV() == 0)
			{
				Launcher.SetLoaded(true);
			}

			var speed = -Controllers.ApplyDeadzone(Controllers.ShooterController.getLeftY()) * 0.7;
			speed = Math.copySign(speed * speed, speed);
			
			if (Controllers.ShooterController.getAButton())
			{
				speed = -0.1;
				Launcher.GroundIntakeMotor.set(-0.1);
				Launcher.IntakeMotor.set(-0.1);
			}
			else if (Controllers.ShooterController.getYButton())
			{
				Launcher.IntakeMotor.set(0.5);
				Launcher.GroundIntakeMotor.set(0.5);
			}
			else if (Controllers.ShooterController.getRightBumperPressed())
			{
				System.out.println("Picking up a Note!");
				GameCommands.IntakeNote().schedule();
				return;
			}
			else
			{
				Launcher.GroundIntakeMotor.stopMotor();
				Launcher.IntakeMotor.stopMotor();
			}
			
			Launcher.SetLaunchSpeed(speed);

		}, Launcher));

		Arm.setDefaultCommand(Commands.run(() ->
		{
			if (Controllers.ShooterController.getXButtonPressed())
			{
				// Arm.RunRotate(Rotation2d.fromDegrees(55)).schedule();
			}
			else
			{
				Arm.SetAngle(Arm.GetTargetAngle().plus(Rotation2d.fromDegrees(-Controllers.ApplyDeadzone(Controllers.ShooterController.getRightY()))));	
			}

			// if (Controllers.ShooterController.getBackButtonPressed())
			// {
			// 	System.out.println("Arm Encoder Pos Reset");
			// 	Arm.Encoder.reset();
			// }

		}, Arm));
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
	public void testInit() 
	{
		Launcher.PerformSysID().schedule();
	}
	
	@Override
	public void disabledInit()
	{
		this.Stop();
	}
}
