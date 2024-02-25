/*
 * Thanks NOMADs
 */

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePivot extends SubsystemBase{
  private TalonFX intakeRotateMotor = new TalonFX(0);
  CurrentLimitsConfigs supplyLimit = new CurrentLimitsConfigs();
  public IntakePivot() {
    supplyLimit.withStatorCurrentLimit(35);
    supplyLimit.withSupplyCurrentLimit(40);
    intakeRotateMotor.getConfigurator().apply(supplyLimit, 0.05);
  }

  public void setAngle(double angle, double armAngle) {
    intakeRotateMotor.setPosition((angle/360*18.5)+(armAngle));
  }
}
