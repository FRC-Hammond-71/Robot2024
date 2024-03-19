package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.utilities.Rotation2dUtils;

public enum ArmPosition 
{
    /**
     * Moves the arm into an up-right position, 90 degrees.
     */
    Default(() -> Rotation2d.fromDegrees(90)),
    /**
     * The identity of the actual arm rotation.
     */
    Identity(() -> Robot.Arm.GetAngle()),
    /**
     * Moves the arm into Amp shooting position.
     */
    Amp(() -> Rotation2d.fromDegrees(100)),
    /**
     * Moves the arm into the intake position.
     */
    Intake(() -> Rotation2d.fromDegrees(50)),
    /**
     * Moves the arm to track the speaker shooting angle.
     */
    TrackingSpeaker(() ->
    {
        double distanceFromSpeaker = Robot.Localization.GetEstimatedPose180().getTranslation().getDistance(FieldConstants.GetSpeakerPosition());
        
        final double heightOfSpeakerIntake = Units.inchesToMeters(83);
        final double shooterYOffset = Units.feetToMeters(1.8);
        
        double rotationRadians = Math.atan((heightOfSpeakerIntake - shooterYOffset) / distanceFromSpeaker);

        return Rotation2d.fromRadians(rotationRadians);
    }),
    BySideSpeaker(() -> Rotation2d.fromDegrees(57)),
    AcrossMap(() -> Rotation2d.fromDegrees(40));

    public final Supplier<Rotation2d> AngleSupplier;

    private ArmPosition(Supplier<Rotation2d> rotation)
    {
        this.AngleSupplier = rotation;
    }

    public Rotation2d GetAngle()
    {
        return Rotation2dUtils.Clamp(this.AngleSupplier.get(), Constants.Arm.MinAngle, Constants.Arm.MaxAngle);
    }
}