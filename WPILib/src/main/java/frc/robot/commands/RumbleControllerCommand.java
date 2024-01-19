package frc.robot.commands;

import java.time.Duration;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RumbleControllerCommand extends CommandBase 
{
    private Duration Duration;
    private XboxController Controller;

    public RumbleControllerCommand(XboxController controller, Duration duration)
    {
        super();

        this.Duration = duration;
        this.Controller = controller;
    }    
}
