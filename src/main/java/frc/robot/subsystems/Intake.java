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

public class Intake extends SubsystemBase {
    public enum IntakeVelocity {
        UNJAM(-25),
        COLLECT(80);

        public double rotationsPerSecond;

        private IntakeVelocity(final double rotationsPerSecond) {
            this.rotationsPerSecond = rotationsPerSecond;
        }
    }

    private final TalonFX leftMotor = new TalonFX(7);
    private final TalonFX rightMotor = new TalonFX(8);
    private final Follower followerControl = new Follower(leftMotor.getDeviceID(), MotorAlignmentValue.Opposed);
    private final VelocityVoltage velocityControl = new VelocityVoltage(0).withSlot(0);
    private final double CLOSED_LOOP_TOLERANCE = 0.0;

    public Intake() {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

        Slot0Configs velocityGains = new Slot0Configs()
                .withKS(0.1)
                .withKV(0.11)
                .withKP(0.5)
                .withKI(0)
                .withKD(0);

        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        talonFXConfiguration.Slot0 = velocityGains;
        talonFXConfiguration.MotorOutput = motorOutputConfigs;

        leftMotor.getConfigurator().apply(talonFXConfiguration);
        leftMotor.getVelocity().setUpdateFrequency(Frequency.ofBaseUnits(100, Hertz));

        rightMotor.getConfigurator().apply(talonFXConfiguration);
        rightMotor.getVelocity().setUpdateFrequency(Frequency.ofBaseUnits(100, Hertz));
        rightMotor.setControl(followerControl);
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

    public Command cmdSetMotorVelocity(IntakeVelocity velocity) {
        return cmdSetMotorVelocity(velocity.rotationsPerSecond);
    }

    public Command cmdStopMotor() {
        return Commands.runOnce(() -> stopMotor(), this);
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
}
