// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
    private final LED systemLights = new LED();
    private final CommandXboxController LightButtons = new CommandXboxController(0);
    public final CommandFactory commandFactory = new CommandFactory(
            swerve,
            List.of(limelight, limelightRear),
            systemLights);

    public RobotContainer() {
        configureBindings();
        LightButtons.a().whileTrue(commandFactory.cmdColorControl(LED.kGreen));
        LightButtons.b().whileTrue(commandFactory.cmdColorControl(LED.kRed));
        LightButtons.x().whileTrue(commandFactory.cmdColorControl(LED.kBlue));
        LightButtons.y().whileTrue(commandFactory.cmdColorControl(LED.kYellow));
        LightButtons.rightTrigger().whileTrue(commandFactory.cmdAnimationControl());
    }

    private void configureBindings() {
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
