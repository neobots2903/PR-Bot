package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ShooterIOSpark implements ShooterIO {
  private final SparkMax shooterMotor; // Spark Max, Neo 1.1
  private final WPI_TalonSRX ballReleaseMotor; // Talon SRX, 775 pro

  private SparkClosedLoopController shooterController;
  private RelativeEncoder shooterEncoder;
  private double targetVelocity = 0.0;

  private boolean isReleasing = false;
  private final Timer releaseTimer = new Timer();
  private final double RELEASE_TIME = 1.0; // 1 second release time

  public ShooterIOSpark(int shooterMotorPort, int ballReleaseMotorPort) {
    this.shooterMotor = new SparkMax(shooterMotorPort, MotorType.kBrushless);
    this.ballReleaseMotor = new WPI_TalonSRX(ballReleaseMotorPort);

    // Shooter Motor Configuration
    shooterEncoder = shooterMotor.getEncoder();
    shooterController = shooterMotor.getClosedLoopController();
    // Configure PID gains as needed for your application
    // shooterController.setP(kP);
    // shooterController.setI(kI);
    // shooterController.setD(kD);
    // shooterController.setFF(kFF);

    // Release Motor Configuration
    ballReleaseMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Update the inputs with current values
    inputs.positionRad = shooterEncoder.getPosition();
    inputs.velocityRadPerSec = shooterEncoder.getVelocity();
    inputs.appliedVolts = shooterMotor.getAppliedOutput() * shooterMotor.getBusVoltage();
    inputs.currentAmps = shooterMotor.getOutputCurrent();
    inputs.targetVelocity = targetVelocity; // Update target velocity
    
    // Check if we need to stop the ball release motor
    if (isReleasing && releaseTimer.hasElapsed(RELEASE_TIME)) {
      ballReleaseMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
      isReleasing = false;
      releaseTimer.stop();
    }
  }

  @Override
  public void setVoltageShooter(double volts) {
    shooterMotor.setVoltage(volts);
  }

  @Override
  public void setVelocityShooter(double velocity) {
    targetVelocity = velocity;
    shooterController.setReference(velocity, SparkMax.ControlType.kVelocity);
  }

  @Override
  public void releaseBall() {
    if (!isReleasing) {
      isReleasing = true;
      releaseTimer.reset();
      releaseTimer.start();
      ballReleaseMotor.set(TalonSRXControlMode.PercentOutput, 1.0);
    }
  }
  
  /**
   * Gets the target velocity
   * @return the target velocity in rad/s
   */
  public double getTargetVelocity() {
    return targetVelocity;
  }
}
