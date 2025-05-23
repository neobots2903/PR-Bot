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

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

/**
 * This drive implementation is for Talon SRXs driving brushed motors (e.g. CIMS) WITHOUT encoders.
 */
public class DriveIOTalonSRX implements DriveIO {
  private final TalonSRX leftLeader = new TalonSRX(leftLeaderCanId);
  private final TalonSRX leftFollower = new TalonSRX(leftFollowerCanId);
  private final TalonSRX rightLeader = new TalonSRX(rightLeaderCanId);
  private final TalonSRX rightFollower = new TalonSRX(rightFollowerCanId);

  public DriveIOTalonSRX() {
    var config = new TalonSRXConfiguration();
    config.peakCurrentLimit = currentLimit;
    config.continuousCurrentLimit = currentLimit - 15;
    config.peakCurrentDuration = 250;
    config.voltageCompSaturation = 12.0;

    tryUntilOkV5(5, () -> leftLeader.configAllSettings(config));
    tryUntilOkV5(5, () -> leftFollower.configAllSettings(config));
    tryUntilOkV5(5, () -> rightLeader.configAllSettings(config));
    tryUntilOkV5(5, () -> rightFollower.configAllSettings(config));

    leftLeader.setInverted(leftInverted);
    rightLeader.setInverted(rightInverted);

    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    inputs.leftAppliedVolts = leftLeader.getMotorOutputVoltage();
    inputs.leftCurrentAmps =
        new double[] {leftLeader.getStatorCurrent(), leftFollower.getStatorCurrent()};

    inputs.rightAppliedVolts = rightLeader.getMotorOutputVoltage();
    inputs.rightCurrentAmps =
        new double[] {rightLeader.getStatorCurrent(), rightFollower.getStatorCurrent()};
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    // OK to just divide by 12 because voltage compensation is enabled
    leftLeader.set(TalonSRXControlMode.PercentOutput, leftVolts / 12.0);
    rightLeader.set(TalonSRXControlMode.PercentOutput, rightVolts / 12.0);
  }
}
