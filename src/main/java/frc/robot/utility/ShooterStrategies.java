package frc.robot.utility;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.Hood.HoodPosition;
import frc.robot.utility.FieldLocalization.Landmark;
import frc.robot.utility.FieldLocalization.Zone;

import java.util.ArrayList;
import java.util.List;

public class ShooterStrategies {

    public static List<ShooterStrategy> strategies = new ArrayList<>();

    static {
        strategies.add(
                new ShooterStrategy(
                        Alliance.Red,
                        Zone.RED_ALLIANCE_ZONE,
                        Landmark.RED_HUB,
                        HoodPosition.ALLIANCE_ZONE,
                        10));

        strategies.add(
                new ShooterStrategy(
                        Alliance.Red,
                        Zone.BLUE_DEPOT_RED_OUTPOST_NEUTRAL_ZONE,
                        Landmark.RED_OUTPOST,
                        HoodPosition.NEUTRAL_ZONE,
                        20));

        strategies.add(
                new ShooterStrategy(
                        Alliance.Red,
                        Zone.BLUE_OUTPOST_RED_DEPOT_NEUTRAL_ZONE,
                        Landmark.RED_DEPOT,
                        HoodPosition.NEUTRAL_ZONE,
                        20));

        strategies.add(
                new ShooterStrategy(
                        Alliance.Blue,
                        Zone.BLUE_ALLIANCE_ZONE,
                        Landmark.BLUE_HUB,
                        HoodPosition.ALLIANCE_ZONE,
                        10));

        strategies.add(
                new ShooterStrategy(
                        Alliance.Blue,
                        Zone.BLUE_DEPOT_RED_OUTPOST_NEUTRAL_ZONE,
                        Landmark.BLUE_DEPOT,
                        HoodPosition.NEUTRAL_ZONE,
                        20));

        strategies.add(
                new ShooterStrategy(
                        Alliance.Blue,
                        Zone.BLUE_OUTPOST_RED_DEPOT_NEUTRAL_ZONE,
                        Landmark.BLUE_OUTPOST,
                        HoodPosition.NEUTRAL_ZONE,
                        20));
    }
}
