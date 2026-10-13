// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.List;
import java.util.Optional;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
import frc.robot.utility.FieldLocalization.ShootingLocation;

public class RobotContainer {
    private final Swerve swerve = TunerConstants.createDrivetrain();
    private final Limelight limelight = new Limelight("limelight", LimelightLocalizationMode.ENABLED);

    @Logged(name = "Shooter")
    private final Shooter shooter = new Shooter();

    // @Logged(name = "Hood")
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

    private final Telemetry logger = new Telemetry();

    private double timeLeftInShift = 0;
    private Shift shift = Shift.AUTO;
    private Optional<Time> timeOpt;
    private Optional<Shift> shiftOpt;

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    private final Trigger robotIsAligned = new Trigger(() -> targetManager.getTargetingState().isAligned());

    private final Trigger robotInRange = new Trigger(() -> {
        var distance = getTargetDistance();
        return distance >= Units.feetToMeters(5.5) && distance <= Units.feetToMeters(6.5);
    });

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
        swerve.registerTelemetry(logger::telemeterize);
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
        return autoChooser.getSelected();
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

    private double getTargetDistance() {
        return targetManager.getTargetingState().distance();
    }

    private void configureButtonBindings() {
        swerve.setDefaultCommand(
                swerve.applyRequest(() -> drive
                        .withVelocityX(yLimiter.calculate(-primary.getLeftY() * MaxSpeed))
                        .withVelocityY(xLimiter.calculate(-primary.getLeftX() * MaxSpeed))
                        .withRotationalRate(-primary.getRightX() * MaxAngularRate))
                        .withName("Field centric"));

        primary.start().onTrue(swerve.runOnce(() -> swerve.seedFieldCentric()).withName("Seed field centric"));

        primary.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
        primary.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));

        /*
         * Joystick Y = quasistatic forward
         * Joystick A = quasistatic reverse
         * Joystick B = dynamic forward
         * Joystick X = dyanmic reverse
         */
        primary.povUp().whileTrue(swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        primary.povDown().whileTrue(swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        primary.povLeft().whileTrue(swerve.sysIdDynamic(SysIdRoutine.Direction.kForward));
        primary.povRight().whileTrue(swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse));
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
        addPathAuto("LeftTrenchSingleSweep", "LeftTrenchSingleSweep");
        addPathAuto("LeftTrenchSingleSweepAndDepot", "LeftTrenchSingleSweepAndDepot");
        addPathAuto("LeftTrenchDoubleSweep", "LeftTrenchDoubleSweep");
        addPathAuto("RightTrenchSingleSweep", "RightTrenchSingleSweep");
        addPathAuto("RightTrenchDoubleSweep", "RightTrenchDoubleSweep");
        addPathAuto("CenterDepotOutpost", "CenterDepotOutpost");

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureNamedCommands() {
        NamedCommands.registerCommand("FireFuel",
                commandFactory.cmdFireFuel(
                        ShooterVelocity.SHOOT,
                        HoodPosition.ALLIANCE_ZONE).withTimeout(4)
                        .andThen(commandFactory.cmdStopFireFuel()));

        NamedCommands.registerCommand("FireFuelByDistance",
                commandFactory.cmdFireFuel(
                        () -> getVelocityBasedOnTargetDistance(),
                        () -> getHoodPositionBasedOnZone().rotations).withTimeout(4)
                        .andThen(commandFactory.cmdStopFireFuel()));

        NamedCommands.registerCommand("CollectFuel", commandFactory.cmdCollectFuel());

        NamedCommands.registerCommand("WarmShooter",
                shooter.cmdSetMotorVelocity(ShooterVelocity.SHOOT));
    }
}
