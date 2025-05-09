package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  
  // Constants for the shooter
  private static final double SHOOTER_SPEED_TOLERANCE = 0.1; // rad/s tolerance

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  /**
   * Run the shooter at the specified percentage of full power
   * @param percent Percentage of full power (-1.0 to 1.0)
   * @return Command that runs the shooter and stops it when the command ends
   */
  public Command runPercent(double percent) {
    return runEnd(
        () -> io.setVoltageShooter(12.0 * percent),
        () -> io.setVoltageShooter(0.0));
  }

  /**
   * Run the shooter at the specified velocity
   * @param velocity Target velocity in rad/s
   * @return Command that runs the shooter and stops it when the command ends
   */
  public Command runVelocity(double velocity) {
    return runEnd(
        () -> io.setVelocityShooter(velocity),
        () -> io.setVelocityShooter(0.0));
  }

  /**
   * Release a ball by spinning the release wheel
   * @return Command that starts the ball release process
   */
  public Command releaseBall() {
    return this.runOnce(() -> io.releaseBall());
  }

  /**
   * Waits until the shooter reaches target speed, then releases the ball
   * @return Command sequence that waits for speed then releases
   */
  public Command shootWhenAtSpeed() {
    return new WaitUntilCommand(() -> isAtTargetSpeed())
        .andThen(releaseBall());
  }
  
  /**
   * Checks if the shooter is at the target speed within tolerance
   * @return true if at target speed, false otherwise
   */
  public boolean isAtTargetSpeed() {    
    return Math.abs(inputs.velocityRadPerSec - inputs.targetVelocity) < SHOOTER_SPEED_TOLERANCE;
  }
}