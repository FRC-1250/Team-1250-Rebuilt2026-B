// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;

public class Loader extends SubsystemBase {
    private TalonFX LoaderPlaceholder1 = new TalonFX(1);
    private TalonFX LoaderPlaceholder2 = new TalonFX(3);

    /** Creates a new Loader. */
    public Loader() {
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
