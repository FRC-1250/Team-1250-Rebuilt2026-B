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

public class Intake extends SubsystemBase {

    public enum IntakeVelocity {
        UNJAM(-25),
        GO(80);

        // All placeholder values
        public double rotationsPerSecond;

        private IntakeVelocity(final double rotationsPerSecond) {
            this.rotationsPerSecond = rotationsPerSecond;
        }
    }

    private final TalonFX leftMotor = new TalonFX(7);
    private final TalonFX rightMotor = new TalonFX(8);
    private final Follower followerControl = new Follower(leftMotor.getDeviceID(), MotorAlignmentValue.Opposed);
    private final VelocityVoltage velocityVoltageControl = new VelocityVoltage(0).withSlot(0);

    public Intake() {
        configureIntake();
    }

    public void setIntakeVelocity(final double rotationsPerSecond) {
        leftMotor.setControl(
                velocityVoltageControl
                        .withVelocity(rotationsPerSecond)
                        .withFeedForward(Volts.of(0)));
    }

    public boolean isIntakeNearRotationsPerSecond(final double rotationsPerSecond, final double tolerance) {
        return leftMotor.getVelocity().isNear(rotationsPerSecond, tolerance);
    }

    public void stop() {
        leftMotor.stopMotor();
    }

    @Logged(name = "Left Motor Velocity")
    public double getLeftMotorVelocity() {
        return leftMotor.getVelocity().getValueAsDouble();
    }

    @Logged(name = "Left Motor Stator Current")
    public double getLeftMotorStatorCurrent() {
        return leftMotor.getStatorCurrent().getValueAsDouble();
    }

    @Logged(name = "Left Motor Supply Current")
    public double getLeftMotorSupplyCurrent() {
        return leftMotor.getSupplyCurrent().getValueAsDouble();
    }

    @Logged(name = "Right Motor Velocity")
    public double getRightMotorVelocity() {
        return rightMotor.getVelocity().getValueAsDouble();
    }

    @Logged(name = "Right Motor Stator Current")
    public double getRightMotorStatorCurrent() {
        return rightMotor.getStatorCurrent().getValueAsDouble();
    }

    @Logged(name = "Right Motor Supply Current")
    public double getRightMotorSupplyCurrent() {
        return rightMotor.getSupplyCurrent().getValueAsDouble();
    }

    private void configureIntake() {
        final MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

        final Slot0Configs velocityGains = new Slot0Configs()
                .withKS(0.1)
                .withKV(0.11)
                .withKP(0.5)
                .withKI(0)
                .withKD(0);

        final TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        talonFXConfiguration.Slot0 = velocityGains;
        talonFXConfiguration.MotorOutput = motorOutputConfigs;

        leftMotor.getConfigurator().apply(talonFXConfiguration);
        leftMotor.getVelocity().setUpdateFrequency(Frequency.ofBaseUnits(100, Hertz));

        rightMotor.getConfigurator().apply(talonFXConfiguration);
        rightMotor.getVelocity().setUpdateFrequency(Frequency.ofBaseUnits(100, Hertz));
        rightMotor.setControl(followerControl);
    }
}
