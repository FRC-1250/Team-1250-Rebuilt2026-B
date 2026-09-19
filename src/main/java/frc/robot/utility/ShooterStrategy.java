package frc.robot.utility;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.Hood.HoodPosition;
import frc.robot.utility.FieldLocalization.Landmark;
import frc.robot.utility.FieldLocalization.Zone;

public final class ShooterStrategy {
    private final Alliance alliance;
    private final Zone zone;
    private final Landmark landmark;
    private final HoodPosition hoodPosition;
    private final int priority;

    public ShooterStrategy(Alliance alliance, Zone zone, Landmark landmark, HoodPosition hoodPosition, int priority) {
        this.alliance = alliance;
        this.zone = zone;
        this.landmark = landmark;
        this.hoodPosition = hoodPosition;
        this.priority = priority;
    }

    public Alliance getAlliance() {
        return alliance;
    }

    public Zone getZone() {
        return zone;
    }

    public Landmark getLandmark() {
        return landmark;
    }

    public int getPriority() {
        return priority;
    }

    public HoodPosition getHoodPosition() {
        return hoodPosition;
    }

}
