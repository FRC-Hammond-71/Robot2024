package frc.robot.subsystems;

import java.io.Console;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

// https://github.com/Mechanical-Advantage/RobotCode2024/blob/main/src/main/java/org/littletonrobotics/frc2024/subsystems/superstructure/arm/ArmVisualizer.java
public class ArmVisualization implements Sendable
{
	private final Mechanism2d Mechanism;
	private final MechanismLigament2d Arm;
	private final MechanismLigament2d TargetArm;
	private final MechanismLigament2d Note;

	public ArmVisualization()
	{
		Mechanism = new Mechanism2d(3.0, 3.0, new Color8Bit(Color.kBlack));

		MechanismRoot2d root = Mechanism.getRoot("pivot", 0.8, 0.4);
		Arm = new MechanismLigament2d("arm", 2, 90, 7, new Color8Bit(Color.kWhite));
		TargetArm = new MechanismLigament2d("target_arm", 1, 90, 4, new Color8Bit(Color.kGray));
		root.append(TargetArm);
		root.append(Arm);
		root.append(new MechanismLigament2d("forward_limit_arm", 2, Constants.Arm.MinAngle.getDegrees(), 4, new Color8Bit(Color.kDarkRed)));
		root.append(new MechanismLigament2d("backward_limit_arm", 2, Constants.Arm.MaxAngle.getDegrees(), 4, new Color8Bit(Color.kDarkRed)));

		var top = Mechanism.getRoot("top-bar", 0.5, 2.8);
		Note = new MechanismLigament2d("note", 2, 0, 10, new Color8Bit(Color.kOrange));
		top.append(Note);	
	}

	/** Update arm visualizer with current arm angle */
	public void Update(ArmSubsystem arm)
	{
		Arm.setAngle(arm.GetAngle());
		TargetArm.setAngle(arm.Mode.GetAngle());

		// Note.setAngle(arm.GetAngle());
		Note.setColor(new Color8Bit(Robot.Launcher.IsLoaded() ? Color.kOrange : Color.kBlack));
	}

	@Override
	public void initSendable(SendableBuilder builder)
	{
		this.Mechanism.initSendable(builder);
	}
}
