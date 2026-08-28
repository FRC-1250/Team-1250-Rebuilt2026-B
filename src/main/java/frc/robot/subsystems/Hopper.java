// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {
    public enum HopperPosition {
        MIN(0),
        HOME(0.25),
        EXTENDED(4),
        MAX(4.05);

        public double rotations;

        private HopperPosition(double rotations) {
            this.rotations = rotations;
        }

    }

    private final TalonFX hopperMotor = new TalonFX(30);
    private final DigitalInput homeSensor = new DigitalInput(1);
    private final PositionVoltage hopperPositionVoltage = new PositionVoltage(0).withSlot(1);
    private final PositionVoltage hopperHoldPosition = new PositionVoltage(0).withSlot(2);

    public Hopper() {
        configureMotionMagicHopper();
    }

    public void setPosition(double rotations) {
        hopperMotor.setControl(
                hopperPositionVoltage
                        .withPosition(rotations)
                        .withFeedForward(Volts.of(0)));
    }

    public void holdPosition() {
        var pos = hopperMotor.getPosition().waitForUpdate(0.02);
        hopperMotor.setControl(
                hopperHoldPosition
                        .withPosition(pos.getValueAsDouble()));
    }

    public void resetPosition(double rotations) {
        hopperMotor.setPosition(rotations);
    }

    public void stop() {
        hopperMotor.stopMotor();
    }

    public boolean isNearAmps(double current, double tolerance) {
        return hopperMotor.getStatorCurrent().isNear(current, tolerance);
    }

    public boolean isNearPosition(double rotations, double tolerance) {
        return hopperMotor.getPosition().isNear(rotations, tolerance);
    }

    @Logged(name = "Sensor state")
    public boolean getSensorState() {
        return homeSensor.get();
    }

    @Logged(name = "Motor Position")
    public double getMotorPosition() {
        return hopperMotor.getPosition().getValueAsDouble();
    }

    @Logged(name = "Motor Stator current")
    public double getMotorStatorCurrent() {
        return hopperMotor.getStatorCurrent().getValueAsDouble();
    }

    @Logged(name = "Motor Supply current")
    public double getMotorSupplyCurrent() {
        return hopperMotor.getSupplyCurrent().getValueAsDouble();
    }

    private void configureMotionMagicHopper() {
        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        talonFXConfiguration.Voltage.PeakReverseVoltage = -12;
        talonFXConfiguration.Voltage.PeakForwardVoltage = 12;

        MotorOutputConfigs motorOutputConfigs = talonFXConfiguration.MotorOutput;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

        Slot1Configs positionGains = talonFXConfiguration.Slot1;
        positionGains.GravityType = GravityTypeValue.Elevator_Static;
        positionGains.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        positionGains.kS = 0.4; // output to overcome static friction (output)
        positionGains.kV = 0.15; // output per unit of target velocity (output/rps)
        positionGains.kG = -0.7;
        positionGains.kA = 0; // output per unit of target acceleration (output/(rps/s))
        positionGains.kP = 2; // output per unit of error in position (output/rotation)
        positionGains.kI = 0; // output per unit of integrated error in position (output/(rotation*s))
        positionGains.kD = 0.01; // output per unit of error in velocity (output/rps)

        Slot2Configs holdConfigs = talonFXConfiguration.Slot2;
        holdConfigs.kP = 20;

        CurrentLimitsConfigs currentLimitsConfigs = talonFXConfiguration.CurrentLimits;
        currentLimitsConfigs.SupplyCurrentLimit = 20;
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;

        SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs = talonFXConfiguration.SoftwareLimitSwitch;
        softwareLimitSwitchConfigs.ForwardSoftLimitEnable = true;
        softwareLimitSwitchConfigs.ForwardSoftLimitThreshold = 4;
        softwareLimitSwitchConfigs.ReverseSoftLimitEnable = true;
        softwareLimitSwitchConfigs.ReverseSoftLimitThreshold = 0;

        MotionMagicConfigs motionMagicConfigs = talonFXConfiguration.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 8;
        motionMagicConfigs.MotionMagicAcceleration = 64;
        motionMagicConfigs.MotionMagicAcceleration = 256;

        hopperMotor.getConfigurator().apply(talonFXConfiguration);
        hopperMotor.setPosition(0);
        hopperMotor.getPosition().setUpdateFrequency(Frequency.ofBaseUnits(100, Hertz));
    }

}
