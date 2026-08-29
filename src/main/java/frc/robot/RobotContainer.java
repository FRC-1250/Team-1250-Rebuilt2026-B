// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.LimelightLocalizationMode;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.utility.HubTracker;
import frc.robot.utility.HubTracker.Shift;
import frc.robot.utility.RobotLocalization;
import frc.robot.utility.TargetManager;

public class RobotContainer {
    private final Swerve swerve = TunerConstants.createDrivetrain();
    private final Limelight limelight = new Limelight("limelight", LimelightLocalizationMode.ENABLED);

    @Logged(name = "Shooter")
    private final Shooter shooter = new Shooter();

    @Logged(name = "Hood")
    private final Hood hood = new Hood();

    @Logged(name = "Loader")
    private final Loader loader = new Loader();

    @Logged(name = "Intake")
    private final Intake intake = new Intake();

    @Logged(name = "Indexer")
    private final Indexer indexer = new Indexer();

    @Logged(name = "Hopper")
    private final Hopper hopper = new Hopper();

    private final TargetManager targetManager = new TargetManager();
    private final RobotLocalization robotLocalization = new RobotLocalization(List.of(limelight), swerve);

    private final CommandXboxController primaryDriver = new CommandXboxController(0);

    private final double SHIFT_CLOCK_WARNING = 8.0;
    private final double SHIFT_CLOCK_PRE_FIRE = 2.0;
    private double timeLeftInShift = 0;
    private Shift shift = Shift.AUTO;
    private Optional<Time> timeOpt;
    private Optional<Shift> shiftOpt;

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
            () -> HubTracker.isActive());

    public RobotContainer() {
        configureRumbleProfiles();
        configureBindings();
    }

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

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    public void updateTargetState() {
        targetManager.updateTargetState(
                robotLocalization.getActiveZones(),
                DriverStation.getAlliance().orElse(Alliance.Blue),
                swerve.getState(),
                swerve.getOperatorForwardDirection());
        SmartDashboard.putString("Targeting State", targetManager.getTargetingState().toString());
    }

    public void updateVisionState() {
        robotLocalization.processMegaTag2Measurement();
        robotLocalization.processActiveZone();
    }

    private void configureBindings() {

    }

    private void configureRumbleProfiles() {

    }

    private Rotation2d getRotationToTargetBasedOnZone() {
        return targetManager.getTargetingState().rotation();
    }

    private double getVelocityBasedOnTargetDistance() {
        return shooter.getInterpolatedVelocity(targetManager.getTargetingState().distance());
    }
}
