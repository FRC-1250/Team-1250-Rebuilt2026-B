package frc.robot;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.List;

import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;
import frc.robot.utility.RobotLocalization;
import frc.robot.utility.TargetManager;

public class CommandFactory {

    private final TargetManager targetManager;
    private final CommandSwerveDrivetrain swerve;
    private final RobotLocalization robotLocalization;
    private final LED systemLights;

    public CommandFactory(
            CommandSwerveDrivetrain swerve,
            List<Limelight> limelights,
            LED systemLights) {
        this.swerve = swerve;
        this.systemLights = systemLights;
        this.targetManager = new TargetManager();
        this.robotLocalization = new RobotLocalization(limelights, swerve);
    }

    /*
     * Drive
     */

    public void updateTargetState() {
        targetManager.updateTargetState(
                robotLocalization.getActiveZones(),
                DriverStation.getAlliance().orElse(Alliance.Blue),
                swerve.getState(),
                swerve.getOperatorForwardDirection());
        SmartDashboard.putString("Targeting State", targetManager.getTargetingState().toString());
    }

    private Command driveTest(double targetVelocityX, double targetVelocityY, double duration, int steps) {
        SequentialCommandGroup sequence = new SequentialCommandGroup();
        SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        double stepVelocityX = targetVelocityX / steps;
        double stepVelocityY = targetVelocityY / steps;
        double stepDuration = duration / steps;

        for (int i = 1; i <= steps; i++) {
            final double currentX = stepVelocityX * i;
            final double currentY = stepVelocityY * i;

            sequence.addCommands(
                    swerve.applyRequest(() -> drive
                            .withVelocityX(currentX)
                            .withVelocityY(currentY)
                            .withRotationalRate(0))
                            .withTimeout(stepDuration));
        }

        sequence.addCommands(
                swerve.applyRequest(() -> drive
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0))
                        .withTimeout(2));

        return sequence;
    }

    private Command rotateTest(double targetRotationRate, double duration, int steps) {
        SequentialCommandGroup sequence = new SequentialCommandGroup();
        SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        double stepRotationalRate = targetRotationRate / steps;
        double stepDuration = duration / steps;

        for (int i = 1; i <= steps; i++) {
            final double currentRate = stepRotationalRate * i;

            sequence.addCommands(
                    swerve.applyRequest(() -> drive
                            .withVelocityX(0)
                            .withVelocityY(0)
                            .withRotationalRate(currentRate))
                            .withTimeout(stepDuration));
        }

        sequence.addCommands(
                swerve.applyRequest(() -> drive
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0))
                        .withTimeout(2));

        return sequence;
    }

    private Command pointWheelsTest(double targetVelocity, double duration, int steps, boolean clockwise) {
        SequentialCommandGroup sequence = new SequentialCommandGroup();
        SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        double stepAngle = (2 * Math.PI) / steps;
        double stepDuration = duration / steps;

        if (clockwise) {
            stepAngle = -stepAngle;
        }

        for (int index = 0; index < steps; index++) {
            double theta = index * stepAngle;
            double x = Math.cos(theta);
            double y = Math.sin(theta);
            double magnitude = Math.hypot(x, y);

            final double velocityX;
            final double velocityY;

            if (magnitude > 0) {
                velocityX = (x / magnitude) * targetVelocity;
                velocityY = (y / magnitude) * targetVelocity;
            } else {
                velocityX = 0;
                velocityY = 0;
            }

            sequence.addCommands(
                    swerve.applyRequest(() -> drive
                            .withVelocityX(velocityX)
                            .withVelocityY(velocityY)
                            .withRotationalRate(0))
                            .withTimeout(stepDuration));
        }

        sequence.addCommands(
                swerve.applyRequest(() -> drive
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0))
                        .withTimeout(2));

        return sequence;
    }

    public Command driveProveOut() {
        final double targetTestVelocity = 2.5;
        final double targetTestRotationRate = RotationsPerSecond.of(0.375).in(RadiansPerSecond);

        final int driveTestSteps = 3;
        final double driveTestDuration = 5;

        final int rotateTestSteps = 3;
        final double rotateTestDuration = 5;

        final int pointWheelsTestSteps = 10;
        final double pointWheelsTestDuration = 5;

        return Commands.sequence(
                driveTest(targetTestVelocity, 0, driveTestDuration, driveTestSteps),
                driveTest(-targetTestVelocity, 0, driveTestDuration, driveTestSteps),
                driveTest(0, targetTestVelocity, driveTestDuration, driveTestSteps),
                driveTest(0, -targetTestVelocity, driveTestDuration, driveTestSteps),
                rotateTest(targetTestRotationRate, rotateTestDuration, rotateTestSteps),
                rotateTest(-targetTestRotationRate, rotateTestDuration, rotateTestSteps),
                pointWheelsTest(targetTestVelocity, pointWheelsTestDuration, pointWheelsTestSteps, false),
                pointWheelsTest(targetTestVelocity, pointWheelsTestDuration, pointWheelsTestSteps, true));
    }

    public Command cmdColorControl(RGBWColor newColor) {
        return Commands.run(() -> systemLights.ColorControl(newColor, 0, 8), systemLights);
    }

    public Command cmdAnimationControl() {
        return Commands.run(() -> systemLights.AnimationControl(), systemLights);
    }

    public Command proveOut() {
        return Commands.sequence(
                driveProveOut());
    }
}
