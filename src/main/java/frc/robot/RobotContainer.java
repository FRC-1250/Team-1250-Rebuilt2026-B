// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.List;
import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
import frc.robot.subsystems.Shooter.ShooterVelocity;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Hood.HoodPosition;
import frc.robot.utility.HubTracker;
import frc.robot.utility.HubTracker.Shift;
import frc.robot.utility.RobotLocalization;
import frc.robot.utility.ShooterStrategyManager;

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
    public final CommandFactory commandFactory = new CommandFactory(
            hood,
            hopper,
            indexer,
            intake,
            loader,
            shooter);

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final ShooterStrategyManager targetManager = new ShooterStrategyManager();
    private final RobotLocalization robotLocalization = new RobotLocalization(List.of(limelight), swerve);

    private final CommandXboxController primary = new CommandXboxController(0);
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(20, -20, 0);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(20, -20, 0);

    private final double SHIFT_CLOCK_WARNING = 8.0;
    private final double SHIFT_CLOCK_PRE_FIRE = 2.0;
    private double timeLeftInShift = 0;
    private Shift shift = Shift.AUTO;
    private Optional<Time> timeOpt;
    private Optional<Shift> shiftOpt;

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

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

    private final Trigger robotIsAligned = new Trigger(() -> targetManager.getTargetingState().isAligned());

    private final FieldCentric drive = new FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final FieldCentricFacingAngle driveWithAngle = new FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public RobotContainer() {
        configureButtonBindings();
        configureNamedCommands();
        configureAutoCommands();
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

    private Rotation2d getRotationToTargetBasedOnZone() {
        return targetManager.getTargetingState().targetAngle();
    }

    private double getVelocityBasedOnTargetDistance() {
        return targetManager.getTargetingState().shooterVelocity();
    }

    private HoodPosition getHoodPositionBasedOnZone() {
        return targetManager.getTargetingState().hoodPosition();
    }

    private void configureButtonBindings() {
        swerve.setDefaultCommand(
                swerve.applyRequest(() -> drive
                        .withVelocityX(yLimiter.calculate(-primary.getLeftY() * MaxSpeed))
                        .withVelocityY(xLimiter.calculate(-primary.getLeftX() * MaxSpeed))
                        .withRotationalRate(-primary.getRightX() * MaxAngularRate))
                        .withName("Field centric"));

        primary.start().onTrue(swerve.runOnce(() -> swerve.seedFieldCentric()).withName("Seed field centric"));

        primary.rightTrigger().and(primary.leftTrigger()).and(robotIsAligned)
                .whileTrue(commandFactory.cmdFireFuel(
                        () -> getVelocityBasedOnTargetDistance(),
                        () -> getHoodPositionBasedOnZone().rotations)
                        .withName("Shoot by distance"));

        primary.rightTrigger().and(primary.leftTrigger().negate())
                .whileTrue(commandFactory.cmdFireFuel(
                        ShooterVelocity.TOWER,
                        HoodPosition.ALLIANCE_ZONE)
                        .withName("Shoot default"));
                        
        primary.rightTrigger().negate().and(primary.leftTrigger().negate())
                .whileTrue(hood.cmdSetMotorPosition(HoodPosition.HOME));

        primary.leftTrigger().whileTrue(
                swerve.applyRequest(
                        () -> driveWithAngle
                                .withVelocityX(yLimiter.calculate(-primary.getLeftY() * (MaxSpeed * 0.5)))
                                .withVelocityY(xLimiter.calculate(-primary.getLeftX() * (MaxSpeed * 0.5)))
                                .withHeadingPID(15, 0, 0)
                                .withTargetDirection(getRotationToTargetBasedOnZone()))
                        .withName("Field centric facing angle"));

        primary.b().whileTrue(
                swerve.applyRequest(
                        () -> driveWithAngle
                                .withVelocityX(yLimiter.calculate(-primary.getLeftY() * (MaxSpeed * 0.33)))
                                .withVelocityY(xLimiter.calculate(-primary.getLeftX() * (MaxSpeed * 0.33)))
                                .withHeadingPID(0, 0, 0)
                                .withTargetDirection(swerve.getOperatorForwardDirection().plus(Rotation2d.k180deg)))
                        .withName("Snap backwards"));

        primary.a().whileTrue(
                swerve.applyRequest(
                        () -> driveWithAngle
                                .withVelocityX(yLimiter.calculate(-primary.getLeftY() * (MaxSpeed * 0.33)))
                                .withVelocityY(xLimiter.calculate(-primary.getLeftX() * (MaxSpeed * 0.33)))
                                .withHeadingPID(15, 0, 0)
                                .withTargetDirection(swerve.getOperatorForwardDirection()))
                        .withName("Snap forward"));

        primary.rightBumper().onTrue(commandFactory.cmdCollectFuel().withName("Collect fuel"));
        primary.leftBumper().onTrue(commandFactory.cmdStopCollectFuel().withName("Stop collect fuel"));
    }

    private void addPathAuto(String name, String pathName) {
        try {
            autoChooser.addOption(name, new PathPlannerAuto(pathName));
        } catch (Exception e) {
            // Exceptions are now caught in the PathPlannerAuto constructor and this should
            // never run. Leaving it in place to catch any edge cases.
            DataLogManager.log(String.format("GatorBot: Not able to build auto routines! %s", e.getMessage()));
        }
    }

    private void configureAutoCommands() {
        /*
         * Do nothing as default is a human safety condition, this should always be the
         * default
         */
        autoChooser.setDefaultOption("Do nothing", new WaitCommand(15));
        addPathAuto("LeftTrenchSweep", "LeftTrenchSweep");
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureNamedCommands() {
        NamedCommands.registerCommand("FireFuel",
                commandFactory.cmdFireFuel(
                        ShooterVelocity.TOWER,
                        HoodPosition.ALLIANCE_ZONE).withTimeout(3));
    }
}
