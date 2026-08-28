package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Frequency;

public class Hood {
    public enum HoodPosition {
        HOME(0.0),
        EXTENDED(0.0),
        THREAD(0.0);

        public double rotations;

        private HoodPosition(final double rotations) {
            this.rotations = rotations;
        }
    }

    private final TalonFX hoodMotor = new TalonFX(35);
    private final CANcoder hoodEncoder = new CANcoder(36);
    private final PositionVoltage positionVoltageControl = new PositionVoltage(0).withSlot(1);
    private final double MAGNET_OFFSET = 0.2;

    public Hood() {
        configureHood();
    }

    public void setPosition(final double rotations) {
        hoodMotor.setControl(
                positionVoltageControl
                        .withPosition(rotations)
                        .withFeedForward(Volts.of(0)));

    }

    public void stop() {
        hoodMotor.stopMotor();
    }

    public boolean isNearPosition(final double rotations, final double tolerance) {
        return hoodEncoder.getAbsolutePosition().isNear(rotations, tolerance);
    }

    @Logged(name = "Encoder Abs position")
    public double getEncoderAbsolutePosition() {
        return hoodEncoder.getAbsolutePosition().getValueAsDouble();
    }

    @Logged(name = "Motor Stator current")
    public double getMotorStatorCurrent() {
        return hoodMotor.getStatorCurrent().getValueAsDouble();
    }

    @Logged(name = "Motor Supply current")
    public double getMotorSupplyCurrent() {
        return hoodMotor.getSupplyCurrent().getValueAsDouble();
    }

    private void configureHood() {
        final TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();

        final MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

        final Slot1Configs positionGains = new Slot1Configs()
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                .withKS(0.25)
                .withKP(8)
                .withKI(0)
                .withKD(0.01);
        talonFXConfiguration.Feedback.FeedbackRemoteSensorID = hoodEncoder.getDeviceID();
        talonFXConfiguration.Slot1 = positionGains;
        talonFXConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        talonFXConfiguration.Feedback.RotorToSensorRatio = 34.7826;
        talonFXConfiguration.MotorOutput = motorOutputConfigs;

        hoodMotor.getConfigurator().apply(talonFXConfiguration);
        hoodMotor.getPosition().setUpdateFrequency(Frequency.ofBaseUnits(100, Hertz));
        hoodMotor.getRotorPosition().setUpdateFrequency(Frequency.ofBaseUnits(100, Hertz));

        final CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
        canCoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        canCoderConfiguration.MagnetSensor.MagnetOffset = MAGNET_OFFSET;
        canCoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        hoodEncoder.getConfigurator().apply(canCoderConfiguration);
        hoodEncoder.getPosition().setUpdateFrequency(Frequency.ofBaseUnits(100, Hertz));
        hoodEncoder.getAbsolutePosition().setUpdateFrequency(Frequency.ofBaseUnits(100, Hertz));
    }
}
