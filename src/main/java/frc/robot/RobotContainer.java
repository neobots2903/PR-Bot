// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveIOTalonSRX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSpark;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  // private final Roller roller;
  private final Shooter shooter;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  // private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // drive = new Drive(new DriveIOTalonSRX(), new GyroIOPigeon2());
        drive = new Drive(new DriveIOTalonSRX());
        shooter = new Shooter(new ShooterIOSpark(1, 10));
        // roller = new Roller(new RollerIOTalonSRX());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        // drive = new Drive(new DriveIOSim(), new GyroIO() {});
        drive = new Drive(new DriveIOSim());
        shooter = new Shooter(new ShooterIOSpark(100, 100)); // Fix Later
        // roller = new Roller(new RollerIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        // drive = new Drive(new DriveIO() {}, new GyroIO() {});
        drive = new Drive(new DriveIO() {});
        shooter = new Shooter(new ShooterIO() {});
        // roller = new Roller(new RollerIO() {});
        break;
    }

    // Set up auto routines
    // NamedCommands.registerCommand("Score", roller.runPercent(1.0).withTimeout(3.0));
    // autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default drive command, normal arcade drive
    drive.setDefaultCommand(
        DriveCommands.arcadeDrive(
            drive, () -> -controller.getLeftY(), () -> -controller.getRightX()));

    // Default roller command, control with triggers
    // roller.setDefaultCommand(
    //     roller.runTeleop(
    //         () -> controller.getRightTriggerAxis(), () -> controller.getLeftTriggerAxis()));

    // Shooter commands (right trigger to shoot, passes in analog value clamped between 0 and 0.6).
    controller.a().onTrue(Commands.runOnce(() -> shooter.releaseBall()));

    controller.rightTrigger().whileTrue(
        Commands.run(() -> shooter.runPercent(Math.min(Math.pow(controller.getRightTriggerAxis(), 2) * Math.signum(controller.getRightTriggerAxis()), 0.6))));
    // shooter.runPercent(Math.min(Math.copySign(controller.getRightTriggerAxis() * controller.getRightTriggerAxis(), controller.getRightTriggerAxis()), 0.6));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return autoChooser.get();
    return null;
  }
}
