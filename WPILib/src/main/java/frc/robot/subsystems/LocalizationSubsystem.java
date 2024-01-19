package frc.robot.subsystems;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LocalizationSubsystem extends SubsystemBase {
    
    private DriveSubsystem m_drive;
    
    private Translation2d m_position = new Translation2d();

    public LocalizationSubsystem(DriveSubsystem drive) 
    {
        this.m_drive = drive;
    }

    @Override
    public void periodic() 
    {
        // TODO: Use time.
        
        // Wheel Specs:
        // Radius = 3 in
        // Circumference = Radius * 2 * PI = 18.85 inches = 0.48 meters

        

    }

}