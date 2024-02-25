// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.IntakePivot;

public class TeleopArm extends Command {
  private DoubleSupplier intakeAngleSup;
  private DoubleSupplier armAngleSup;
  private Arm arm;
  private IntakePivot intake;
  /** Creates a new TeleopArm. */
  public TeleopArm(Arm arm, IntakePivot intake, DoubleSupplier intakeAngleSup, DoubleSupplier armAngleSup) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm, intake);
    this.intakeAngleSup = intakeAngleSup;
    this.armAngleSup = armAngleSup;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double intakeAngleVal = intakeAngleSup.getAsDouble();
    double armAngleVal = armAngleSup.getAsDouble();
    arm.setPIDFF(armAngleVal, 5);
    intake.setAngle(intakeAngleVal, armAngleVal);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
