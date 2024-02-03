package frc.robot;

import frc.robot.commands.FaceAtCommand;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.DriveSubsystem;

public class Robot extends TimedRobot {
	// ----------
	// Subsystems
	// ----------
	private DriveSubsystem Drive;

	// private final SendableChooser<String> m_chooser = new SendableChooser<String>();

	@Override
	public void simulationInit() 
	{
		System.out.println("Robot is running in simulation!");
	}

	@Override
	public void robotInit() 
	{
		this.Drive = new DriveSubsystem();

		CameraServer.startAutomaticCapture();

		Shuffleboard.getTab("Movement").add(Drive);
		// Shuffleboard.getTab("General").add(CommandScheduler.getInstance());
	}

	@Override
	public void robotPeriodic() 
	{
		CommandScheduler.getInstance().run();

		// Emergency stop on Driver Controller.
		if (Controllers.DriverController.getPOV() == 180) 
		{
			this.EmergencyStop();
		}
	}

	public void EmergencyStop()
	{
		CommandScheduler.getInstance().cancelAll();
		this.Drive.Stop();
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
	public void disabledInit() 
	{
		
	}

	@Override
	public void disabledPeriodic() 
	{
		this.Drive.Stop();
	}
}
