package frc.robot.subsystems;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Movement.DriveSubsystem;

public class LocalizationSubsystem extends SubsystemBase 
{
    public LocalizationSubsystem(DriveSubsystem drive)
    {
        super();
    }

    @Override
    public void periodic() 
    {
        
    }
    
}
