// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.LimelightLocalizationMode;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utility.HubTracker;
import frc.robot.utility.HubTracker.Shift;

public class RobotContainer {

    /* Subsystems */
    private final CommandSwerveDrivetrain swerve = TunerConstants.createDrivetrain();
    private final Limelight limelight = new Limelight("limelight", LimelightLocalizationMode.ENABLED);
    private final Limelight limelightRear = new Limelight("limelight-rear", LimelightLocalizationMode.DISABED);
    private final LED systemLights = new LED();
    private final CommandXboxController LightButtons = new CommandXboxController(0);
    public final CommandFactory commandFactory = new CommandFactory(
            swerve,
            List.of(limelight, limelightRear),
            systemLights);

    private final double SHIFT_CLOCK_WARNING = 8.0;
    private final double SHIFT_CLOCK_PRE_FIRE = 2.0;
    private double timeLeftInShift = 0;
    private Shift shift = Shift.AUTO;
    private Optional<Time> timeOpt;
    private Optional<Shift> shiftOpt;

    public Shift getShift() {
        return shift;
    }

    public double getTimeLeftInShift() {
        return timeLeftInShift;
    }

    public void processShiftClock() {
        timeOpt = HubTracker.timeRemainingInCurrentShift();
        if (timeOpt.isPresent()) {
            timeLeftInShift = timeOpt.get().baseUnitMagnitude();
        } else {
            timeLeftInShift = Double.MAX_VALUE;
        }

        shiftOpt = HubTracker.getCurrentShift();
        if (shiftOpt.isPresent()) {
            shift = shiftOpt.get();
        } else {
            shift = Shift.AUTO;
        }
    }

    private final Trigger hubInactive = new Trigger(
            () -> (timeLeftInShift > SHIFT_CLOCK_WARNING
                    && !HubTracker.isActive()));

    private final Trigger hubActiveSoon = new Trigger(
            () -> (timeLeftInShift <= SHIFT_CLOCK_WARNING
                    && timeLeftInShift > SHIFT_CLOCK_PRE_FIRE
                    && !HubTracker.isActive()));

    private final Trigger hubActivePreFire = new Trigger(
            () -> (timeLeftInShift <= SHIFT_CLOCK_PRE_FIRE
                    && !HubTracker.isActive()));

    private final Trigger hubActive = new Trigger(
            () -> !HubTracker.isActive());

    public RobotContainer() {
        configureBindings();
        LightButtons.a().whileTrue(commandFactory.cmdColorControl(LED.kGreen));
        LightButtons.b().whileTrue(commandFactory.cmdColorControl(LED.kRed));
        LightButtons.x().whileTrue(commandFactory.cmdColorControl(LED.kBlue));
        LightButtons.y().whileTrue(commandFactory.cmdColorControl(LED.kYellow));
        LightButtons.rightTrigger().whileTrue(commandFactory.cmdAnimationControl());
        hubActiveSoon.onTrue(Commands.run(() -> LightButtons.setRumble(RumbleType.kBothRumble, 0.5)));
    }

    private void configureBindings() {
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }pulse=new

    VibrationProfile
    pulse.addStep(new VibrationStep(1,0.25));
}
