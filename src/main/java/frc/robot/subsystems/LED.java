// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    /** Creates a new LED. */
    public CANdle m_candle = new CANdle(30, CANBus.roboRIO());
    public static final RGBWColor kGreen = new RGBWColor(0, 217, 0, 0);
    public static final RGBWColor kRed = new RGBWColor(217, 0, 0, 0);
    public static final RGBWColor kBlue = new RGBWColor(0, 0, 217, 0);
    public static final RGBWColor kYellow = new RGBWColor(255, 255, 0, 0);

    public LED() {

    }

    public void ColorControl(RGBWColor newColor, int startint, int endint) {
        m_candle.setControl(new SolidColor(startint, endint).withColor(newColor));
    }

    public void ColorControl(RGBWColor newColor) {
        ColorControl(newColor, 0, 8);
    }

    public void AnimationControl() {
        m_candle.setControl(new RainbowAnimation(0, 8));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
