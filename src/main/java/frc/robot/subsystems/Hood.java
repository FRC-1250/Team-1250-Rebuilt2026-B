package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
    public enum HoodPosition {
        REVERSE_LIMIT(0.0),
        HOME(0.0),
        ALLIANCE_ZONE(0.0),
        NEUTRAL_ZONE(0.0),
        OPPOSSING_ALLIANCE_ZONE(0.0),
        FORWARD_LIMIT(0.0);

        public double rotations;

        private HoodPosition(double rotations) {
            this.rotations = rotations;
        }
    }

    private final TalonFX hoodMotor = new TalonFX(35);
    private final CANcoder hoodEncoder = new CANcoder(36);
    private final PositionVoltage positionControl = new PositionVoltage(0).withSlot(0);
    private final double MAGNET_OFFSET = 0.2;
    private final double CLOSED_LOOP_TOLERANCE = 0.5;

    public Hood() {
        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();

        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

        Slot0Configs positionGains = new Slot0Configs()
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                .withKS(0.25)
                .withKP(8)
                .withKI(0)
                .withKD(0.01);
        talonFXConfiguration.Feedback.FeedbackRemoteSensorID = hoodEncoder.getDeviceID();
        talonFXConfiguration.Slot0 = positionGains;
        talonFXConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        talonFXConfiguration.Feedback.RotorToSensorRatio = 34.7826;
        talonFXConfiguration.MotorOutput = motorOutputConfigs;

        hoodMotor.getConfigurator().apply(talonFXConfiguration);
        hoodMotor.getPosition().setUpdateFrequency(Frequency.ofBaseUnits(100, Hertz));
        hoodMotor.getRotorPosition().setUpdateFrequency(Frequency.ofBaseUnits(100, Hertz));

        CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
        canCoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        canCoderConfiguration.MagnetSensor.MagnetOffset = MAGNET_OFFSET;
        canCoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        hoodEncoder.getConfigurator().apply(canCoderConfiguration);
        hoodEncoder.getPosition().setUpdateFrequency(Frequency.ofBaseUnits(100, Hertz));
        hoodEncoder.getAbsolutePosition().setUpdateFrequency(Frequency.ofBaseUnits(100, Hertz));
    }

    public void setMotorPosition(double rotations) {
        hoodMotor.setControl(positionControl.withPosition(rotations));
    }

    public void stopMotor() {
        hoodMotor.stopMotor();
    }

    public boolean isMotorAtPosition(double rotations) {
        return hoodEncoder.getAbsolutePosition().isNear(rotations, CLOSED_LOOP_TOLERANCE);
    }

    public Command cmdSetMotorPosition(double rotations) {
        return Commands.runOnce(() -> setMotorPosition(rotations), this).until(() -> isMotorAtPosition(rotations));
    }

    public Command cmdSetMotorPosition(HoodPosition position) {
        return cmdSetMotorPosition(position.rotations);
    }

    public Command cmdStopMotor() {
        return Commands.runOnce(() -> stopMotor(), this);
    }

    @Logged(name = "Encoder Abs Position")
    public double getEncoderAbsolutePosition() {
        return hoodEncoder.getAbsolutePosition().getValueAsDouble();
    }

    @Logged(name = "Motor Position")
    public double getMotorPosition() {
        return hoodMotor.getPosition().getValueAsDouble();
    }

    @Logged(name = "Motor Stator Current")
    public double getMotorStatorCurrent() {
        return hoodMotor.getStatorCurrent().getValueAsDouble();
    }

    @Logged(name = "Motor Supply Current")
    public double getMotorSupplyCurrent() {
        return hoodMotor.getSupplyCurrent().getValueAsDouble();
    }
}
