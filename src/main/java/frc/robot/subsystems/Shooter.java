// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
 // private CANSparkFlex shooterRight = new CANSparkFlex(3, MotorType.kBrushless);
 // private CANSparkFlex shooterLeft = new CANSparkFlex(4, MotorType.kBrushless);
  /** Creates a new Shooter. */
  public Shooter() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public void shoot(double speed) {
    //shooterLeft.set(-speed);
    //shooterRight.set(speed);
  }
}
