package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Indexer.IndexerVelocity;
import frc.robot.subsystems.Intake.IntakeVelocity;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Loader.LoaderVelocity;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Hood.HoodPosition;
import frc.robot.subsystems.Hopper.HopperPosition;
import frc.robot.subsystems.Shooter.ShooterVelocity;

public class CommandFactory {
    private final Hood hood;
    private final Hopper hopper;
    private final Indexer indexer;
    private final Intake intake;
    private final Loader loader;
    private final Shooter shooter;

    public CommandFactory(
            Hood hood,
            Hopper hopper,
            Indexer indexer,
            Intake intake,
            Loader loader,
            Shooter shooter) {
        this.hood = hood;
        this.hopper = hopper;
        this.indexer = indexer;
        this.intake = intake;
        this.loader = loader;
        this.shooter = shooter;
    }

    public Command cmdFireFuel(DoubleSupplier shooterVelocitySupplier, DoubleSupplier hoodPositionSupplier) {
        return Commands.run(() -> {
            double shooterVelocity = shooterVelocitySupplier.getAsDouble();
            double hoodPosition = hoodPositionSupplier.getAsDouble();

            shooter.setMotorVelocity(shooterVelocity);
            hood.setMotorPosition(hoodPosition);

            if (shooter.isMotorAtVelocity(shooterVelocity) && hood.isMotorAtPosition(hoodPosition)) {
                loader.setMotorVelocity(LoaderVelocity.LOAD.rotationsPerSecond);
                indexer.setMotorVelocity(IndexerVelocity.LOAD.rotationsPerSecond);
            } else {
                loader.stopMotor();
                indexer.stopMotor();
            }
        }, shooter, loader, indexer);
    }

    public Command cmdFireFuel(ShooterVelocity shooterVelocity, HoodPosition hoodPosition) {
        return Commands.parallel(
                shooter.cmdSetMotorVelocity(shooterVelocity),
                hood.cmdSetMotorPosition(hoodPosition))
                .andThen(
                        Commands.run(() -> {
                            if (shooter.isMotorAtVelocity(shooterVelocity.rotationsPerSecond)
                                    && hood.isMotorAtPosition(hoodPosition.rotations)) {
                                loader.setMotorVelocity(LoaderVelocity.LOAD.rotationsPerSecond);
                                indexer.setMotorVelocity(IndexerVelocity.LOAD.rotationsPerSecond);
                            } else {
                                loader.stopMotor();
                                indexer.stopMotor();
                            }
                        }, shooter, loader, indexer));

    }

    public Command cmdStopFireFuel() {
        return Commands.run(() -> {
            shooter.setMotorVelocity(ShooterVelocity.WARM.rotationsPerSecond);
            loader.stopMotor();
            indexer.stopMotor();
        }, shooter, loader, indexer);
    }

    public Command cmdCollectFuel() {
        return Commands.sequence(
                hopper.cmdSetMotorPosition(HopperPosition.DEPLOYED),
                intake.cmdSetMotorVelocity(IntakeVelocity.COLLECT));
    }

    public Command cmdStopCollectFuel() {
        return Commands.sequence(
                intake.cmdStopMotor(),
                hopper.cmdSetMotorPosition(HopperPosition.HOME));
    }
}
