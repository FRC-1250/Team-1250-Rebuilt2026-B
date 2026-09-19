package frc.robot.utility;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.Hood.HoodPosition;
import frc.robot.subsystems.Shooter.ShooterVelocity;
import frc.robot.utility.FieldLocalization.Landmark;
import frc.robot.utility.FieldLocalization.Zone;

public class ShooterStrategyManager {

    public record ShooterState(
            HoodPosition hoodPosition,
            double shooterVelocity,
            Rotation2d targetAngle,
            boolean isAligned) {
    }

    private ShooterState activeState = new ShooterState(
            HoodPosition.HOME,
            ShooterVelocity.WARM.rotationsPerSecond,
            Rotation2d.kZero,
            false);

    private final double ANGLE_TOLERANCE = 2.5;

    public ShooterStrategyManager() {
    }

    public void updateTargetState(
            List<Zone> activeZones,
            Alliance alliance,
            SwerveDriveState swerveDriveState,
            Rotation2d operatorForwardDirection) {

        var optActiveStrategy = getStrategy(activeZones, alliance);

        if (optActiveStrategy.isPresent()) {
            var activeStrategy = optActiveStrategy.get();

            Rotation2d fieldRelativeTargetAngle = calculateFieldRelativeTargetAngle(
                    swerveDriveState,
                    activeStrategy.getLandmark());

            Rotation2d operatorRelativeTargetAngle = fieldRelativeTargetAngle.plus(operatorForwardDirection);

            double distance = activeStrategy.getLandmark().location
                    .getDistance(swerveDriveState.Pose.getTranslation());

            double shooterVelocity = calculateShooterVelocity(activeStrategy.getHoodPosition(), distance);

            activeState = new ShooterState(
                    activeStrategy.getHoodPosition(),
                    shooterVelocity,
                    operatorRelativeTargetAngle,
                    isAligned(fieldRelativeTargetAngle, swerveDriveState.Pose.getRotation()));
        } else {
            activeState = new ShooterState(
                    HoodPosition.HOME,
                    ShooterVelocity.WARM.rotationsPerSecond,
                    operatorForwardDirection,
                    false);
        }
    }

    public ShooterState getTargetingState() {
        return activeState;
    }

    private Rotation2d calculateFieldRelativeTargetAngle(SwerveDriveState swerveState, Landmark landmark) {
        return Rotation2d.fromRadians(
                Math.atan2(
                        landmark.location.getY() - swerveState.Pose.getY(),
                        landmark.location.getX() - swerveState.Pose.getX()));
    }

    private boolean isAligned(Rotation2d fieldRelativeTargetAngle, Rotation2d swervePoseRotation) {
        double angleError = Math.abs(swervePoseRotation.minus(fieldRelativeTargetAngle).getDegrees());
        return angleError <= ANGLE_TOLERANCE;
    }

    private double calculateShooterVelocity(HoodPosition hoodPosition, double distance) {
        InterpolatingDoubleTreeMap lookUpTable = ShooterProjectileLookUp.hoodPositionLookUpTableMap
                .get(hoodPosition);

        double velocity = ShooterVelocity.WARM.rotationsPerSecond;
        if (lookUpTable != null) {
            velocity = lookUpTable.get(distance);
        }
        return velocity;
    }

    private Optional<ShooterStrategy> getStrategy(List<Zone> activeZones, Alliance alliance) {
        ShooterStrategy priorityShooterStrategy = null;
        for (ShooterStrategy shooterStrategy : ShooterStrategies.strategies) {
            boolean isMatchingAlliance = shooterStrategy.getAlliance() == alliance;
            boolean isMatchingZone = activeZones.contains(shooterStrategy.getZone());

            if (isMatchingAlliance && isMatchingZone) {
                if (priorityShooterStrategy == null
                        || shooterStrategy.getPriority() > priorityShooterStrategy.getPriority()) {
                    priorityShooterStrategy = shooterStrategy;
                }
            }
        }
        return Optional.ofNullable(priorityShooterStrategy);
    }

}
