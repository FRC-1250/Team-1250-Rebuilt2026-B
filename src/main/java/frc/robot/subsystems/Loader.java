// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
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

public class Loader extends SubsystemBase {
    public enum LoaderVelocity {
        UNJAM(-10),
        LOAD(25);

        public double rotationsPerSecond;

        LoaderVelocity(final double shooterRotationsPerSecond) {
            this.rotationsPerSecond = shooterRotationsPerSecond;
        }
    }

    private final TalonFX left = new TalonFX(4);
    private final TalonFX right = new TalonFX(5);
    private final Follower followerControl = new Follower(left.getDeviceID(), MotorAlignmentValue.Opposed);
    private final VelocityVoltage velocityVoltageControl = new VelocityVoltage(0).withSlot(0);

    public Loader() {
        configureLoader();
    }

    public void setLoaderVelocity(final double rotationsPerSecond) {
        left.setControl(
                velocityVoltageControl
                        .withVelocity(rotationsPerSecond)
                        .withFeedForward(Volts.of(0)));
    }

    public boolean isLoaderNearRotationsPerSecond(final double rotationsPerSecond, final double tolerance) {
        return left.getVelocity().isNear(rotationsPerSecond, tolerance);
    }

    public void stopLoader() {
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

    @Logged(name = "Right Velocity")
    public double getRightVelocity() {
        return right.getVelocity().getValueAsDouble();
    }

    @Logged(name = "Right Stator Current")
    public double getRightStatorCurrent() {
        return right.getStatorCurrent().getValueAsDouble();
    }

    @Logged(name = "Right Supply Current")
    public double getRightSupplyCurrent() {
        return right.getSupplyCurrent().getValueAsDouble();
    }

    private void configureLoader() {
        final MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

        final Slot0Configs velocityGains = new Slot0Configs()
                .withKS(0.09)
                .withKV(0.11)
                .withKP(0.15)
                .withKI(0)
                .withKD(0);

        final TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        talonFXConfiguration.Slot0 = velocityGains;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 50;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfiguration.MotorOutput = motorOutputConfigs;

        left.getConfigurator().apply(talonFXConfiguration);
        left.getVelocity().setUpdateFrequency(Frequency.ofBaseUnits(100, Hertz));

        right.getConfigurator().apply(talonFXConfiguration);
        right.getVelocity().setUpdateFrequency(Frequency.ofBaseUnits(100, Hertz));
        right.setControl(followerControl);
    }
}
