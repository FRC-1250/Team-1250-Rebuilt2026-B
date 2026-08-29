// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {
    public enum HopperPosition {
        REVERSE_LIMIT(0),
        HOME(0.0),
        DEPLOYED(0),
        FORWARD_LIMIT(4.05);

        public double rotations;

        private HopperPosition(double rotations) {
            this.rotations = rotations;
        }

    }

    private final TalonFX hopperMotor = new TalonFX(30);
    private final DigitalInput hopperHomeSensor = new DigitalInput(1);
    private final PositionVoltage positionControl = new PositionVoltage(0).withSlot(0);
    private final double CLOSED_LOOP_TOLERANCE = 0.0;

    public Hopper() {
        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        talonFXConfiguration.Voltage.PeakReverseVoltage = -12;
        talonFXConfiguration.Voltage.PeakForwardVoltage = 12;

        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

        Slot0Configs positionGains = new Slot0Configs();
        positionGains.GravityType = GravityTypeValue.Elevator_Static;
        positionGains.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        positionGains.kS = 0.4; // output to overcome static friction (output)
        positionGains.kV = 0.15; // output per unit of target velocity (output/rps)
        positionGains.kG = -0.7;
        positionGains.kA = 0; // output per unit of target acceleration (output/(rps/s))
        positionGains.kP = 2; // output per unit of error in position (output/rotation)
        positionGains.kI = 0; // output per unit of integrated error in position (output/(rotation*s))
        positionGains.kD = 0.01; // output per unit of error in velocity (output/rps)

        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        currentLimitsConfigs.SupplyCurrentLimit = 20;
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;

        SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs();
        softwareLimitSwitchConfigs.ForwardSoftLimitEnable = true;
        softwareLimitSwitchConfigs.ForwardSoftLimitThreshold = 4;
        softwareLimitSwitchConfigs.ReverseSoftLimitEnable = true;
        softwareLimitSwitchConfigs.ReverseSoftLimitThreshold = 0;

        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = 8;
        motionMagicConfigs.MotionMagicAcceleration = 64;
        motionMagicConfigs.MotionMagicAcceleration = 256;

        talonFXConfiguration.MotorOutput = motorOutputConfigs;
        talonFXConfiguration.Slot0 = positionGains;
        talonFXConfiguration.CurrentLimits = currentLimitsConfigs;
        talonFXConfiguration.SoftwareLimitSwitch = softwareLimitSwitchConfigs;
        talonFXConfiguration.MotionMagic = motionMagicConfigs;

        hopperMotor.getConfigurator().apply(talonFXConfiguration);
        hopperMotor.setPosition(0);
        hopperMotor.getPosition().setUpdateFrequency(Frequency.ofBaseUnits(100, Hertz));
    }

    public void setMotorPosition(double rotations) {
        hopperMotor.setControl(positionControl.withPosition(rotations));
    }

    public void setPosition(double rotations) {
        hopperMotor.setPosition(rotations);
    }

    public void stopMotor() {
        hopperMotor.stopMotor();
    }

    public boolean isMotorAtPosition(double rotations) {
        return hopperMotor.getPosition().isNear(rotations, CLOSED_LOOP_TOLERANCE);
    }

    public Command cmdSetMotorPosition(double rotations) {
        return Commands.runOnce(() -> setMotorPosition(rotations), this).until(() -> isMotorAtPosition(rotations));
    }

    public Command cmdSetMotorPosition(HopperPosition position) {
        return cmdSetMotorPosition(position.rotations);
    }

    public Command cmdStopMotor() {
        return Commands.runOnce(() -> stopMotor(), this);
    }

    public Command cmdResetMotorPosition() {
        return Commands.runOnce(() -> {
            this.setPosition(0);
        }).ignoringDisable(true);
    }

    public Command cmdResetMotorPositionWithSensor() {
        return Commands.sequence(
                Commands.runOnce(() -> hopperMotor.set(-0.2), this),
                Commands.waitUntil(() -> getSensorState()),
                Commands.runOnce(() -> setPosition(HopperPosition.REVERSE_LIMIT.rotations)),
                Commands.runOnce(() -> setMotorPosition(HopperPosition.HOME.rotations)));
    }

    @Logged(name = "Sensor state")
    public boolean getSensorState() {
        return hopperHomeSensor.get();
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
}
