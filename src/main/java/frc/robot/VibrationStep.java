package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class VibrationStep {
    private RumbleType rumbleType;
    private double intensity;
    private double interval;

    public VibrationStep(RumbleType rumbleType, double intensity, double interval) {
        this.rumbleType = rumbleType;
        this.intensity = intensity;
        this.interval = interval;
    }

    public double getIntensity() {
        return intensity;
    }

    public double getInterval() {
        return interval;
    }

    public RumbleType getRumbleType() {
        return rumbleType;
    }
}
