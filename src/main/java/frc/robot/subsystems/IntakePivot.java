/*
 * Thanks NOMADs
 */

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePivot extends SubsystemBase{
  private TalonFX intakeRotateMotor = new TalonFX(9);
  CurrentLimitsConfigs supplyLimit = new CurrentLimitsConfigs();
  final PositionVoltage request = new PositionVoltage(0).withSlot(0);
  public IntakePivot() {
    supplyLimit.withStatorCurrentLimit(15);
    supplyLimit.withSupplyCurrentLimit(20);
    intakeRotateMotor.getConfigurator().apply(supplyLimit, 0.05);
  }

  public void setAngle(double angle, double armAngle) {
    intakeRotateMotor.setControl(request.withPosition((angle/360*13.75)+(armAngle/360*13.75)));
  }
}
