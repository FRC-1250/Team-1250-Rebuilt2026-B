// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

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
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

public class Shooter extends SubsystemBase {

    private TalonFX ShooterPlaceholder1 = new TalonFX(17); // Used as the leader motor placeholder
    private TalonFX ShooterPlaceholder2 = new TalonFX(21);
    private TalonFX ShooterPlaceholder3 = new TalonFX(2); // Used as the follower motor placeholder
    private final Follower ShooterPlaceholder3Control = new Follower(ShooterPlaceholder1.getDeviceID(),
            MotorAlignmentValue.Opposed);

    private final double kGearRatio = 1.0;

    private final DCMotorSim m_motorSimModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    DCMotor.getFalcon500(1), 0.001, kGearRatio),
            DCMotor.getFalcon500(1));

    public enum ShooterVelocity {
        UNJAM(-10),
        WARM(10),
        MIN(40),
        TOWER(48),
        TRENCH(55),
        MAX(80); // Do not go any faster than this

        public double rotationsPerSecond;

        ShooterVelocity(double shooterRotationsPerSecond) {
            this.rotationsPerSecond = shooterRotationsPerSecond;
        }
    }

    private final InterpolatingDoubleTreeMap shooterVelocityLUT = new InterpolatingDoubleTreeMap();
    private final VelocityVoltage shooterVelocityControl = new VelocityVoltage(0).withSlot(0);

    public Shooter() {
        configureShooter();
        configureVelocityMap();

        if (Robot.isSimulation()) {
            simulationInit();
        }
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
        var hubFrontToCenterOffsetMeters = 0.591;
        var robotFrontToCenterOffsetMeters = 0.724 / 2; // with bumpers
        var offset = hubFrontToCenterOffsetMeters + robotFrontToCenterOffsetMeters;

        shooterVelocityLUT.put(Units.feetToMeters(4) + offset, 40.0);
        shooterVelocityLUT.put(Units.feetToMeters(6) + offset, 45.0);
        shooterVelocityLUT.put(Units.feetToMeters(8) + offset, 48.0);
        shooterVelocityLUT.put(Units.feetToMeters(10) + offset, 53.0);
        shooterVelocityLUT.put(Units.feetToMeters(20) + offset, 70.0);
    }

    public double getInterpolatedVelocity(double distance) {
        // get handles interpolation for you here
        return shooterVelocityLUT.get(distance);
    }

    public void setShooterVelocity(double rotationsPerSecond) {
        ShooterPlaceholder1.setControl(
                shooterVelocityControl
                        .withVelocity(rotationsPerSecond)
                        .withFeedForward(Volts.of(0)));
    }

    public boolean isShooterNearRotationsPerSecond(double rotationsPerSecond, double tolerance) {
        return ShooterPlaceholder1.getVelocity().isNear(rotationsPerSecond, tolerance);
    }

    public void simulationInit() {
        var talonFXSim = ShooterPlaceholder1.getSimState();
        talonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
        talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    }

    public void stopShooter() {
        ShooterPlaceholder1.stopMotor();
    }

    @Logged(name = "Shooter velocity")
    public double getShooterVelocity() {
        return ShooterPlaceholder1.getVelocity().getValueAsDouble();
    }

    @Logged(name = "Shooter leader stator current")
    public double getShooterLeaderStatorCurrent() {
        return ShooterPlaceholder1.getStatorCurrent().getValueAsDouble();
    }

    @Logged(name = "Shooter leader supply current")
    public double getShooterLeaderSupplyCurrent() {
        return ShooterPlaceholder1.getSupplyCurrent().getValueAsDouble();
    }

    @Logged(name = "Shooter follower stator current")
    public double getShooterFollowerStatorCurrent() {
        return ShooterPlaceholder3.getStatorCurrent().getValueAsDouble();
    }

    @Logged(name = "Shooter follower supply current")
    public double getShooterFollowerSupplyCurrent() {
        return ShooterPlaceholder3.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public double getTargetVelocity(double distance) {
        // (meters, rps)
        // y = 15.36x (0,0) to (3.125, 48)
        // y = 56x - 127 (3.125, 48) to (3.25, 55)
        return Math.max(Math.min(15.36 * distance, ShooterVelocity.MAX.rotationsPerSecond),
                ShooterVelocity.MIN.rotationsPerSecond);
    }

    @Override
    public void simulationPeriodic() {
        var talonFXSim = ShooterPlaceholder1.getSimState();

        // set the supply voltage of the TalonFX
        talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        // get the motor voltage of the TalonFX
        var motorVoltage = talonFXSim.getMotorVoltageMeasure();

        // use the motor voltage to calculate new position and velocity
        // using WPILib's DCMotorSim class for physics simulation
        m_motorSimModel.setInputVoltage(motorVoltage.in(Volts));
        m_motorSimModel.update(0.020); // assume 20 ms loop time

        // apply the new rotor position and velocity to the TalonFX;
        // note that this is rotor position/velocity (before gear ratio), but
        // DCMotorSim returns mechanism position/velocity (after gear ratio)
        talonFXSim.setRawRotorPosition(m_motorSimModel.getAngularPosition().times(kGearRatio));
        talonFXSim.setRotorVelocity(m_motorSimModel.getAngularVelocity().times(kGearRatio));
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

        ShooterPlaceholder1.getConfigurator().apply(talonFXConfiguration);
        ShooterPlaceholder1.getVelocity().setUpdateFrequency(Frequency.ofBaseUnits(100, Hertz));

        ShooterPlaceholder3.getConfigurator().apply(talonFXConfiguration);
        ShooterPlaceholder3.getVelocity().setUpdateFrequency(Frequency.ofBaseUnits(100, Hertz));
        ShooterPlaceholder3.setControl(ShooterPlaceholder3Control);
    }
}
