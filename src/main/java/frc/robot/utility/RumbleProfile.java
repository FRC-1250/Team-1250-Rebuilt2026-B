package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;

public class RumbleProfile {

    private Timer timer;
    private List<RumbleStep> steps;
    private int index;

    public RumbleProfile() {
        this.steps = new ArrayList<>();
        this.timer = new Timer();
        index = 0;
    }

    public void addStep(RumbleStep as) {
        if (as.getInterval() > 0)
            this.steps.add(as);
    }

    public void reset() {
        timer.stop();
        timer.reset();
        index = 0;
    }

    public double shift() {
        if (!timer.isRunning()) {
            timer.start();
        }
        var currentStep = steps.get(index % steps.size());

        if (timer.advanceIfElapsed(currentStep.getInterval())) {
            index++;
            currentStep = steps.get(index % steps.size());
            timer.reset();
        }

        return currentStep.getIntensity();
    }

}
