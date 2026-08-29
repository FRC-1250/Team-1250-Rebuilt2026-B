// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.EmptyControl;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.Enable5VRailValue;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.VBatOutputModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    public enum Animation {
        RAINBOW,
        LARSON
    }

    public static final RGBWColor kGreen = new RGBWColor(0, 217, 0, 0);
    public static final RGBWColor kRed = new RGBWColor(217, 0, 0, 0);
    public static final RGBWColor kBlue = new RGBWColor(0, 0, 217, 0);
    public static final RGBWColor kYellow = new RGBWColor(255, 255, 0, 0);

    private final RainbowAnimation rainbow = new RainbowAnimation(0, 0);
    private final LarsonAnimation larson = new LarsonAnimation(0, 0);

    private final EmptyControl emptyControl = new EmptyControl();

    private final CANdle m_candle = new CANdle(30, CANBus.roboRIO());

    private final int DEFAULT_INTERNAL_LED_START = 0;
    private final int DEFAULT_INTERNAL_LED_END = 7;
    private final int DEFAULT_EXTERNAL_LED_START = 8;
    private final int ACTUAL_LED_START;
    private final int ACTUAL_LED_END;

    public LED() {
        configureCandle();
        ACTUAL_LED_START = DEFAULT_INTERNAL_LED_START;
        ACTUAL_LED_END = DEFAULT_INTERNAL_LED_END;
    }

    public LED(int ledCount, boolean startFromInternalLEDs) {
        configureCandle();

        if (startFromInternalLEDs) {
            ACTUAL_LED_START = DEFAULT_INTERNAL_LED_START;
        } else {
            ACTUAL_LED_START = DEFAULT_EXTERNAL_LED_START;
        }

        ACTUAL_LED_END = DEFAULT_EXTERNAL_LED_START + ledCount - 1;
    }

    public void turnOff() {
        m_candle.setControl(emptyControl);
    }

    public void setColor(RGBWColor color) {
        setColor(color, ACTUAL_LED_START, ACTUAL_LED_END);
    }

    public void setColorRange(RGBWColor newColor, int startIndex, int endIndex) {
        setColor(newColor, startIndex, endIndex);
    }

    public void fillColorToEnd(RGBWColor newColor, int startIndex) {
        setColor(newColor, startIndex, ACTUAL_LED_END);
    }

    public void fillColorToStart(RGBWColor newColor, int endIndex) {
        setColor(newColor, ACTUAL_LED_START, endIndex);
    }

    public void setAnimation(Animation animation) {
        setAnimation(animation, ACTUAL_LED_START, ACTUAL_LED_END);
    }

    public void setAnimationRange(Animation animation, int startIndex, int endIndex) {
        setAnimation(animation, startIndex, endIndex);
    }

    public void fillAnimationToEnd(Animation animation, int startIndex) {
        setAnimation(animation, startIndex, ACTUAL_LED_END);
    }

    public void fillAnimationToStart(Animation animation, int endIndex) {
        setAnimation(animation, ACTUAL_LED_START, endIndex);
    }

    public Command cmdColorControl(RGBWColor newColor) {
        return Commands.run(() -> setColor(newColor), this);
    }

    public Command cmdAnimationControl() {
        return Commands.run(() -> setAnimation(Animation.RAINBOW), this);
    }

    private void setColor(RGBWColor newColor, int startIndex, int endIndex) {
        startIndex = normalizeIndex(startIndex);
        endIndex = normalizeIndex(endIndex);

        m_candle.setControl(new SolidColor(startIndex, endIndex).withColor(newColor));
    }

    private void setAnimation(Animation animation, int startIndex, int endIndex) {
        startIndex = normalizeIndex(startIndex);
        endIndex = normalizeIndex(endIndex);

        switch (animation) {
            case RAINBOW:
                m_candle.setControl(rainbow
                        .withLEDStartIndex(startIndex)
                        .withLEDEndIndex(endIndex));
                break;
            case LARSON:
                m_candle.setControl(larson
                        .withLEDStartIndex(startIndex)
                        .withLEDEndIndex(endIndex));
                break;
            default:
                break;
        }
    }

    private int normalizeIndex(int index) {
        if (index < ACTUAL_LED_START) {
            index = ACTUAL_LED_START;
        } else if (index > ACTUAL_LED_END) {
            index = ACTUAL_LED_END;
        }
        return index;
    }

    private void configureCandle() {
        CANdleConfiguration candleConfiguration = new CANdleConfiguration();
        candleConfiguration.LED.BrightnessScalar = 0.6;
        candleConfiguration.LED.StripType = StripTypeValue.GRB;
        candleConfiguration.LED.LossOfSignalBehavior = LossOfSignalBehaviorValue.DisableLEDs;

        candleConfiguration.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
        candleConfiguration.CANdleFeatures.VBatOutputMode = VBatOutputModeValue.Off;
        candleConfiguration.CANdleFeatures.Enable5VRail = Enable5VRailValue.Enabled;
        m_candle.getConfigurator().apply(candleConfiguration);
    }
}