package frc.robot;

public class VibrationSteps {
    private double intensity;
    private double interval;
    private VibrationProfile pulse;

    public void VibrationStep(double intensity, double interval) {
        this.intensity = intensity;
        this.interval = interval;
    }

    public double getIntensity() {
        return intensity;
    }

    public double getInterval() {
        return interval;
    }

}
