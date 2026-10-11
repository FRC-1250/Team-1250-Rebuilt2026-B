package frc.robot.utility;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class FieldZoneAnnulus extends FieldZone {
    private final Translation2d target;
    private final double outerRadius;
    private final double innerRadius;

    public FieldZoneAnnulus(
            Translation2d target,
            double innerRadius,
            double outerRadius) {
        this.target = target;
        this.innerRadius = innerRadius;
        this.outerRadius = outerRadius;
    }

    public FieldZoneAnnulus(
            double x,
            double y,
            double innerRadius,
            double outerRadius) {
        this(new Translation2d(x, y), innerRadius, outerRadius);
    }

    @Override
    public boolean isRobotInZone(Pose2d pose) {
        return isRobotInZone(pose.getX(), pose.getY());
    }

    @Override
    public boolean isRobotInZone(double x, double y) {
        var distanceToTarget = distanceToTarget(x, y);
        return distanceToTarget <= outerRadius && distanceToTarget >= innerRadius;
    }

    private double distanceToTarget(double x, double y) {
        return Math.sqrt(Math.pow(target.getX() - x, 2) + Math.pow(target.getY() - y, 2));
    }

}
