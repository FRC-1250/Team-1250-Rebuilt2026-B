// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

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

  public Shooter() {
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
