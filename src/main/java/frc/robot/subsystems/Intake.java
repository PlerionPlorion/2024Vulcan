// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private CANSparkFlex reverseIntakeMotor = new CANSparkFlex(2, MotorType.kBrushless);
    private CANSparkFlex intakeMotor = new CANSparkFlex(3, MotorType.kBrushless);
  /** Creates a new Intake. */
  public Intake() {
    reverseIntakeMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void intake(double speed, boolean motor) {
    if(motor == false) {
    reverseIntakeMotor.set(-speed);
    } else if (motor == true) {
    intakeMotor.set(speed);
    }
  }
  public void manualIntake(double speed) {
    reverseIntakeMotor.set(-speed);
    intakeMotor.set(speed);
  }
  public void shoot(double speed, boolean motor) {
    if(motor == true) {
      reverseIntakeMotor.set(speed);
      } else if (motor == false) {
      intakeMotor.set(-speed);
      }
  }
}
