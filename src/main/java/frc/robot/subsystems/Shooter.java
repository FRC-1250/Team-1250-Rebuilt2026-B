// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Hertz;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Shooter extends SubsystemBase {

    public enum ShooterVelocity {
        UNJAM(-10),
        WARM(10),
        MIN(40),
        TOWER(48),
        TRENCH(55),
        MAX(80); // Do not go any faster than this

        public double rotationsPerSecond;

        ShooterVelocity(final double shooterRotationsPerSecond) {
            this.rotationsPerSecond = shooterRotationsPerSecond;
        }
    }

    private final TalonFX left = new TalonFX(1);
    private final TalonFX upperRight = new TalonFX(2);
    private final TalonFX lowerRight = new TalonFX(3);
    private final InterpolatingDoubleTreeMap velocityLookUpTable = new InterpolatingDoubleTreeMap();
    private final Follower followerControl = new Follower(left.getDeviceID(), MotorAlignmentValue.Opposed);
    private final VelocityVoltage velocityVoltageControl = new VelocityVoltage(0).withSlot(0);

    public Shooter() {
        configureShooter();
        configureVelocityMap();
    }

    public double getInterpolatedVelocity(final double distance) {
        // get handles interpolation for you
        return velocityLookUpTable.get(distance);
    }

    public double getTargetVelocity(final double distance) {
        // (meters, rps)
        // y = 15.36x (0,0) to (3.125, 48)
        // y = 56x - 127 (3.125, 48) to (3.25, 55)
        return Math.max(Math.min(15.36 * distance, ShooterVelocity.MAX.rotationsPerSecond),
                ShooterVelocity.MIN.rotationsPerSecond);
    }

    public void setShooterVelocity(final double rotationsPerSecond) {
        left.setControl(
                velocityVoltageControl
                        .withVelocity(rotationsPerSecond)
                        .withFeedForward(Volts.of(0)));
    }

    public boolean isShooterNearRotationsPerSecond(final double rotationsPerSecond, final double tolerance) {
        return left.getVelocity().isNear(rotationsPerSecond, tolerance);
    }

    public void stopShooter() {
        left.stopMotor();
    }

    @Logged(name = "Left Velocity")
    public double getLeftVelocity() {
        return left.getVelocity().getValueAsDouble();
    }

    @Logged(name = "Left Stator Current")
    public double getLeftStatorCurrent() {
        return left.getStatorCurrent().getValueAsDouble();
    }

    @Logged(name = "Left Supply Current")
    public double getLeftSupplyCurrent() {
        return left.getSupplyCurrent().getValueAsDouble();
    }

    @Logged(name = "Upper Right Velocity")
    public double getUpperRightVelocity() {
        return upperRight.getVelocity().getValueAsDouble();
    }

    @Logged(name = "Upper Right Stator Current")
    public double getUpperRightStatorCurrent() {
        return upperRight.getStatorCurrent().getValueAsDouble();
    }

    @Logged(name = "Upper Right Supply Current")
    public double getUpperRightSupplyCurrent() {
        return upperRight.getSupplyCurrent().getValueAsDouble();
    }

    @Logged(name = "Lower Right Velocity")
    public double getLowerRightVelocity() {
        return lowerRight.getVelocity().getValueAsDouble();
    }

    @Logged(name = "Lower Right Stator Current")
    public double getLowerRightStatorCurrent() {
        return lowerRight.getStatorCurrent().getValueAsDouble();
    }

    @Logged(name = "Lower Right Supply Current")
    public double getLowerRightSupplyCurrent() {
        return lowerRight.getSupplyCurrent().getValueAsDouble();
    }

    private void configureVelocityMap() {
        /*
         * In code we assume center to center for distance!
         * 
         * Assuming the below input LUT values are measured from front of hub to front
         * of robot bumper, the offset should be added to each value to account for the
         * missing distance.
         * 
         */
        final var hubFrontToCenterOffsetMeters = 0.591;
        final var robotFrontToCenterOffsetMeters = 0.724 / 2; // with bumpers
        final var offset = hubFrontToCenterOffsetMeters + robotFrontToCenterOffsetMeters;

        velocityLookUpTable.put(Units.feetToMeters(4) + offset, 40.0);
        velocityLookUpTable.put(Units.feetToMeters(6) + offset, 45.0);
        velocityLookUpTable.put(Units.feetToMeters(8) + offset, 48.0);
        velocityLookUpTable.put(Units.feetToMeters(10) + offset, 53.0);
        velocityLookUpTable.put(Units.feetToMeters(20) + offset, 70.0);
    }

    private void configureShooter() {
        final MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

        final Slot0Configs velocityGains = new Slot0Configs()
                .withKS(0.09)
                .withKV(0.11)
                .withKP(0.25)
                .withKI(0)
                .withKD(0.01);

        final TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        talonFXConfiguration.Slot0 = velocityGains;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 50;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfiguration.MotorOutput = motorOutputConfigs;

        left.getConfigurator().apply(talonFXConfiguration);
        left.getVelocity().setUpdateFrequency(Frequency.ofBaseUnits(100, Hertz));

        lowerRight.getConfigurator().apply(talonFXConfiguration);
        lowerRight.getVelocity().setUpdateFrequency(Frequency.ofBaseUnits(100, Hertz));
        lowerRight.setControl(followerControl);

        upperRight.getConfigurator().apply(talonFXConfiguration);
        upperRight.getVelocity().setUpdateFrequency(Frequency.ofBaseUnits(100, Hertz));
        upperRight.setControl(followerControl);
    }
}
