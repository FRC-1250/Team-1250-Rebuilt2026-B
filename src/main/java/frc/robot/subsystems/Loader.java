// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Frequency;
//This is basically the accelerator I think so use stuff from that
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
    private TalonFX LoaderPlaceholder1 = new TalonFX(1); // Used as the leader placeholder
    private TalonFX LoaderPlaceholder2 = new TalonFX(3); // Used as the follower placeholder
    private final VelocityVoltage loaderVelocityControl = new VelocityVoltage(0).withSlot(0);
    private final Follower LoaderPlaceholder2Control = new Follower(LoaderPlaceholder1.getDeviceID(),
            MotorAlignmentValue.Opposed);

    public void setAcceleratorVelocity(double rotationsPerSecond) {
        LoaderPlaceholder1.setControl(
                loaderVelocityControl
                        .withVelocity(rotationsPerSecond)
                        .withFeedForward(Volts.of(0)));
    }

    public boolean isAcceleratorNearRotationsPerSecond(double rotationsPerSecond, double tolerance) {
        return LoaderPlaceholder1.getVelocity().isNear(rotationsPerSecond, tolerance);
    }

    /** Creates a new Loader. */
    public Loader() {
        configureLoader();
    }

    public void stopAccelerator() {
        LoaderPlaceholder1.stopMotor();
    }

    private void configureLoader() {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

        Slot0Configs velocityGains = new Slot0Configs()
                .withKS(0.09)
                .withKV(0.11)
                .withKP(0.15)
                .withKI(0)
                .withKD(0);

        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        talonFXConfiguration.Slot0 = velocityGains;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 50;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfiguration.MotorOutput = motorOutputConfigs;

        LoaderPlaceholder1.getConfigurator().apply(talonFXConfiguration);
        LoaderPlaceholder1.getVelocity().setUpdateFrequency(Frequency.ofBaseUnits(100, Hertz));

        LoaderPlaceholder2.getConfigurator().apply(talonFXConfiguration);
        LoaderPlaceholder2.getVelocity().setUpdateFrequency(Frequency.ofBaseUnits(100, Hertz));
        LoaderPlaceholder2.setControl(LoaderPlaceholder2Control);
    }

    @Logged(name = "Accelerator velocity")
    public double getAcceleratorVelocity() {
        return LoaderPlaceholder1.getVelocity().getValueAsDouble();
    }

    @Logged(name = "Accelerator leader stator current")
    public double getAcceleratorLeaderStatorCurrent() {
        return LoaderPlaceholder1.getStatorCurrent().getValueAsDouble();
    }

    @Logged(name = "Accelerator leader supply current")
    public double getAcceleratorLeaderSupplyCurrent() {
        return LoaderPlaceholder1.getSupplyCurrent().getValueAsDouble();
    }

    @Logged(name = "Accelerator follower stator current")
    public double getAcceleratorFollowerStatorCurrent() {
        return LoaderPlaceholder2.getStatorCurrent().getValueAsDouble();
    }

    @Logged(name = "Accelerator follower supply current")
    public double getAcceleratorFollowerSupplyCurrent() {
        return LoaderPlaceholder2.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
