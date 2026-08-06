// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;

public class Intake extends SubsystemBase {
    private TalonFX IntakePlaceholder1 = new TalonFX(7);
    private TalonFX IntakePlaceholder2 = new TalonFX(30);

    /** Creates a new Intake. */
    public Intake() {
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
