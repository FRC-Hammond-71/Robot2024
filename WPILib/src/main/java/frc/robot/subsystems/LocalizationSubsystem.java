package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LocalizationSubsystem extends SubsystemBase 
{
    public LocalizationSubsystem()
    {
        super();
    }

    @Override
    public void periodic() 
    {
        
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        
        // builder.setSmartDashboardType("LocalizationSubsystem");

        super.initSendable(builder);

        

        // builder.addBooleanProperty(".hasDefault", () -> getDefaultCommand() != null, null);
        // builder.addStringProperty(
        //     ".default",
        //     () -> getDefaultCommand() != null ? getDefaultCommand().getName() : "none",
        //     null);
        // builder.addBooleanProperty(".hasCommand", () -> getCurrentCommand() != null, null);
        // builder.addStringProperty(
        //     ".command",
        //     () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "none",
        //     null);

    }
    
}
