// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
    private TalonFX IntakePlaceholder1 = new TalonFX(7); // Used as the leader placeholder
    private TalonFX IntakePlaceholder2 = new TalonFX(30); // Used as the follower placeholder
    private final VelocityVoltage IntakePlaceholder1VelocityControl = new VelocityVoltage(0).withSlot(0);
    private final Follower IntakePlaceholder2FollowerControl = new Follower(
            IntakePlaceholder1.getDeviceID(),
            MotorAlignmentValue.Opposed);

    public enum IntakeVelocity {
        UNJAM(-25),
        GO(80);

        // All placeholder values
        public double rotationsPerSecond;

        private IntakeVelocity(double rotationsPerSecond) {
            this.rotationsPerSecond = rotationsPerSecond;
        }
    }

    /** Creates a new Intake. */
    public Intake() {
        configureIntake();
    }

    private void configureIntake() {
        MotorOutputConfigs motorOutputConfigsUpper = new MotorOutputConfigs();
        motorOutputConfigsUpper.NeutralMode = NeutralModeValue.Coast;
        motorOutputConfigsUpper.Inverted = InvertedValue.Clockwise_Positive;

        MotorOutputConfigs motorOutputConfigsLower = new MotorOutputConfigs();
        motorOutputConfigsLower.NeutralMode = NeutralModeValue.Coast;
        motorOutputConfigsLower.Inverted = InvertedValue.Clockwise_Positive;

        Slot0Configs velocityGains = new Slot0Configs()
                .withKS(0.1)
                .withKV(0.11)
                .withKP(0.5)
                .withKI(0)
                .withKD(0);

        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        talonFXConfiguration.Slot0 = velocityGains;
        // Try default limits. Supply = 70 amp, stator = 120 amp, reduce supply to 40
        // amps after 1 second
        // talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 70;
        // talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;

        IntakePlaceholder1.getConfigurator().apply(talonFXConfiguration);
        IntakePlaceholder1.getVelocity().setUpdateFrequency(Frequency.ofBaseUnits(100, Hertz));

        IntakePlaceholder2.getConfigurator().apply(talonFXConfiguration);
        IntakePlaceholder2.getVelocity().setUpdateFrequency(Frequency.ofBaseUnits(100, Hertz));
        IntakePlaceholder2.setControl(IntakePlaceholder2FollowerControl);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
