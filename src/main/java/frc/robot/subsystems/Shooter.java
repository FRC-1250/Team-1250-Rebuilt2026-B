// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    private final TalonFX leftMotor = new TalonFX(1);
    private final TalonFX upperRightMotor = new TalonFX(2);
    private final TalonFX lowerRightMotor = new TalonFX(3);
    private final InterpolatingDoubleTreeMap velocityLookUpTable = new InterpolatingDoubleTreeMap();
    private final Follower followerControl = new Follower(leftMotor.getDeviceID(), MotorAlignmentValue.Opposed);
    private final VelocityVoltage velocityControl = new VelocityVoltage(0).withSlot(0);
    private final double CLOSED_LOOP_TOLERANCE = 5;

    public Shooter() {
        configureShooter();
        configureVelocityMap();
    }

    public double getInterpolatedVelocity(double distance) {
        // get handles interpolation for you
        return velocityLookUpTable.get(distance);
    }

    public double getTargetVelocity(double distance) {
        // (meters, rps)
        // y = 15.36x (0,0) to (3.125, 48)
        // y = 56x - 127 (3.125, 48) to (3.25, 55)
        return Math.max(Math.min(15.36 * distance, ShooterVelocity.MAX.rotationsPerSecond),
                ShooterVelocity.MIN.rotationsPerSecond);
    }

    public void setMotorVelocity(double rotationsPerSecond) {
        leftMotor.setControl(velocityControl.withVelocity(rotationsPerSecond));
    }

    public boolean isMotorAtVelocity(double rotationsPerSecond) {
        return leftMotor.getVelocity().isNear(rotationsPerSecond, CLOSED_LOOP_TOLERANCE);
    }

    public void stopMotor() {
        leftMotor.stopMotor();
    }

    public Command cmdSetMotorVelocity(double rotationsPerSecond) {
        return Commands.runOnce(() -> setMotorVelocity(rotationsPerSecond), this);
    }

    public Command cmdSetMotorVelocity(ShooterVelocity velocity) {
        return cmdSetMotorVelocity(velocity.rotationsPerSecond);
    }

    public Command cmdStopMotor() {
        return Commands.runOnce(() -> stopMotor(), this);
    }

    @Logged(name = "Left motor Velocity")
    public double getLeftMotorVelocity() {
        return leftMotor.getVelocity().getValueAsDouble();
    }

    @Logged(name = "Left motor Stator Current")
    public double getLeftMotorStatorCurrent() {
        return leftMotor.getStatorCurrent().getValueAsDouble();
    }

    @Logged(name = "Left motor Supply Current")
    public double getLeftMotorSupplyCurrent() {
        return leftMotor.getSupplyCurrent().getValueAsDouble();
    }

    @Logged(name = "Upper Right motor Velocity")
    public double getUpperRightMotorVelocity() {
        return upperRightMotor.getVelocity().getValueAsDouble();
    }

    @Logged(name = "Upper Right motor Stator Current")
    public double getUpperRightMotorStatorCurrent() {
        return upperRightMotor.getStatorCurrent().getValueAsDouble();
    }

    @Logged(name = "Upper Right motor Supply Current")
    public double getUpperRightMotorSupplyCurrent() {
        return upperRightMotor.getSupplyCurrent().getValueAsDouble();
    }

    @Logged(name = "Lower Right motor Velocity")
    public double getLowerRightMotorVelocity() {
        return lowerRightMotor.getVelocity().getValueAsDouble();
    }

    @Logged(name = "Lower Right motor Stator Current")
    public double getLowerRightMotorStatorCurrent() {
        return lowerRightMotor.getStatorCurrent().getValueAsDouble();
    }

    @Logged(name = "Lower Right motor Supply Current")
    public double getLowerRightMotorSupplyCurrent() {
        return lowerRightMotor.getSupplyCurrent().getValueAsDouble();
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
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

        Slot0Configs velocityGains = new Slot0Configs()
                .withKS(0.09)
                .withKV(0.11)
                .withKP(0.25)
                .withKI(0)
                .withKD(0.01);

        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        talonFXConfiguration.Slot0 = velocityGains;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 50;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfiguration.MotorOutput = motorOutputConfigs;

        leftMotor.getConfigurator().apply(talonFXConfiguration);
        leftMotor.getVelocity().setUpdateFrequency(Frequency.ofBaseUnits(100, Hertz));

        lowerRightMotor.getConfigurator().apply(talonFXConfiguration);
        lowerRightMotor.getVelocity().setUpdateFrequency(Frequency.ofBaseUnits(100, Hertz));
        lowerRightMotor.setControl(followerControl);

        upperRightMotor.getConfigurator().apply(talonFXConfiguration);
        upperRightMotor.getVelocity().setUpdateFrequency(Frequency.ofBaseUnits(100, Hertz));
        upperRightMotor.setControl(followerControl);
    }
}
