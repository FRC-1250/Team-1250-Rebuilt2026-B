package frc.robot.utility;

import java.util.EnumMap;
import java.util.Map;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Hood.HoodPosition;

public class ShooterProjectileLookUp {

    // Full robot length with bumpers cut in half to get the center
    private static final double ROBOT_FRONT_TO_CENTER_OFFSET_METERS = 0.724 / 2;

    public static final Map<HoodPosition, InterpolatingDoubleTreeMap> hoodPositionLookUpTableMap = new EnumMap<>(
            HoodPosition.class);

    /*
     * All measurements below should be done center to center.
     */
    static {
        add(HoodPosition.ALLIANCE_ZONE, Units.feetToMeters(4), 40);
        add(HoodPosition.ALLIANCE_ZONE, Units.feetToMeters(6), 40);
        add(HoodPosition.ALLIANCE_ZONE, Units.feetToMeters(8), 40);
        add(HoodPosition.ALLIANCE_ZONE, Units.feetToMeters(10), 40);

        add(HoodPosition.NEUTRAL_ZONE, Units.feetToMeters(4), 40);
        add(HoodPosition.NEUTRAL_ZONE, Units.feetToMeters(6), 40);
        add(HoodPosition.NEUTRAL_ZONE, Units.feetToMeters(8), 40);
        add(HoodPosition.NEUTRAL_ZONE, Units.feetToMeters(10), 40);

        add(HoodPosition.OPPOSSING_ALLIANCE_ZONE, Units.feetToMeters(4), 40);
        add(HoodPosition.OPPOSSING_ALLIANCE_ZONE, Units.feetToMeters(6), 40);
        add(HoodPosition.OPPOSSING_ALLIANCE_ZONE, Units.feetToMeters(8), 40);
        add(HoodPosition.OPPOSSING_ALLIANCE_ZONE, Units.feetToMeters(10), 40);
    }

    private static void add(HoodPosition position, double distanceFromBumperMeters, double shooterVelocity) {
        getOrCreateTable(position).put(distanceFromBumperMeters + ROBOT_FRONT_TO_CENTER_OFFSET_METERS, shooterVelocity);
    }

    private static InterpolatingDoubleTreeMap getOrCreateTable(HoodPosition position) {
        if (!hoodPositionLookUpTableMap.containsKey(position)) {
            hoodPositionLookUpTableMap.put(position, new InterpolatingDoubleTreeMap());
        }
        return hoodPositionLookUpTableMap.get(position);
    }
}
