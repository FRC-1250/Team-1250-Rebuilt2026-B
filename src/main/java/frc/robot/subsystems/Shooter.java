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
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    public enum ShooterVelocity {
        UNJAM(-10),
        WARM(10.0),
        MIN(40.0),
        TOWER(48.0),
        TRENCH(55.0),
        MAX(80.0); // Do not go any faster than this

        public double rotationsPerSecond;

        ShooterVelocity(final double shooterRotationsPerSecond) {
            this.rotationsPerSecond = shooterRotationsPerSecond;
        }
    }

    private final TalonFX shooterFollower = new TalonFX(61);
    private final TalonFX shooterLeader = new TalonFX(11);
    private final Follower followerControl = new Follower(shooterFollower.getDeviceID(), MotorAlignmentValue.Opposed);
    private final VelocityVoltage velocityControl = new VelocityVoltage(0).withSlot(0);
    private final double CLOSED_LOOP_TOLERANCE = 5;

    public Shooter() {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

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

        shooterFollower.getConfigurator().apply(talonFXConfiguration);
        shooterFollower.getVelocity().setUpdateFrequency(Frequency.ofBaseUnits(100, Hertz));

        shooterLeader.getConfigurator().apply(talonFXConfiguration);
        shooterLeader.getVelocity().setUpdateFrequency(Frequency.ofBaseUnits(100, Hertz));
        shooterLeader.setControl(followerControl);
    }

    public void setMotorVelocity(double rotationsPerSecond) {
        shooterFollower.setControl(velocityControl.withVelocity(rotationsPerSecond));
    }

    public boolean isMotorAtVelocity(double rotationsPerSecond) {
        return shooterFollower.getVelocity().isNear(rotationsPerSecond, CLOSED_LOOP_TOLERANCE);
    }

    public void stopMotor() {
        shooterFollower.stopMotor();
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

    @Logged(name = "Follower motor Velocity")
    public double getFollowerVelocity() {
        return shooterFollower.getVelocity().getValueAsDouble();
    }

    @Logged(name = "Follower motor Stator Current")
    public double getFollowerStatorCurrent() {
        return shooterFollower.getStatorCurrent().getValueAsDouble();
    }

    @Logged(name = "Follower motor Supply Current")
    public double getFollowerSupplyCurrent() {
        return shooterFollower.getSupplyCurrent().getValueAsDouble();
    }

    @Logged(name = "Leader motor Velocity")
    public double getLeaderssVelocity() {
        return shooterLeader.getVelocity().getValueAsDouble();
    }

    @Logged(name = "Leader motor Stator Current")
    public double getLeaderStatorCurrent() {
        return shooterLeader.getStatorCurrent().getValueAsDouble();
    }

    @Logged(name = "Leader motor Supply Current")
    public double getLeaderSupplyCurrent() {
        return shooterLeader.getSupplyCurrent().getValueAsDouble();
    }

    private void configureShooter() {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

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

        shooterFollower.getConfigurator().apply(talonFXConfiguration);
        shooterFollower.getVelocity().setUpdateFrequency(Frequency.ofBaseUnits(100, Hertz));

        shooterLeader.getConfigurator().apply(talonFXConfiguration);
        shooterLeader.getVelocity().setUpdateFrequency(Frequency.ofBaseUnits(100, Hertz));
        shooterLeader.setControl(followerControl);
    }
}
