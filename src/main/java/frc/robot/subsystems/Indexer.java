// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;

public class Indexer extends SubsystemBase {
    private TalonFX IndexerPlaceholder1 = new TalonFX(12);
    private TalonFX IndexerPlaceholder2 = new TalonFX(13);

    /** Creates a new Indexer. */
    public Indexer() {
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
