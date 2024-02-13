package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

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
		System.out.println("Updating Timed Robot");
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
}
