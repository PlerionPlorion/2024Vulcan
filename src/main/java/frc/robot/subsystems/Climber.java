// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  TalonFX climberFX;
  CurrentLimitsConfigs supplyLimit = new CurrentLimitsConfigs();
  double encoderDouble;
  final PositionVoltage request = new PositionVoltage(0).withSlot(0);
  /** Creates a new Climber. */
  public Climber() {
    climberFX = new TalonFX(10);
    supplyLimit.withStatorCurrentLimit(25);
    supplyLimit.withSupplyCurrentLimit(30);
    climberFX.getConfigurator().apply(supplyLimit, 0.05);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
  public void climb(double speed) {
    encoderDouble += 100 * speed;
    climberFX.setControl(request.withPosition(speed));
  }
}