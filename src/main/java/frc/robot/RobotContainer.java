// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.LimelightLocalizationMode;

public class RobotContainer {

    /* Subsystems */
    private final CommandSwerveDrivetrain swerve = TunerConstants.createDrivetrain();
    private final Limelight limelight = new Limelight("limelight", LimelightLocalizationMode.ENABLED);
    private final Limelight limelightRear = new Limelight("limelight-rear", LimelightLocalizationMode.DISABED);
    private LED systemLights;

    public final CommandFactory commandFactory = new CommandFactory(
            swerve,
            List.of(limelight, limelightRear),
            systemLights);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
