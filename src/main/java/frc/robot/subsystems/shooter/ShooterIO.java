package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double targetVelocity = 0.0;  // Add target velocity to log
  }

  /** Update the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltageShooter(double volts) {}

  /** Run closed loop at the specified velocity */
  public default void setVelocityShooter(double velocity) {}

  /** Run the ball release wheel for 1 second at max speed */
  public default void releaseBall() {}
}