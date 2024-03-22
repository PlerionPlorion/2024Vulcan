// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.IntakePivot;

public class AmpArm extends Command {
  private double intakeAngleSup;
  private double armAngleSup;
  private double climbPosSup;
  private Arm arm;
  private IntakePivot intake;
  private Climber climb;
  private BooleanSupplier ampBoolSup;
  /** Creates a new TeleopArm. */
  public AmpArm(Arm arm, IntakePivot intake, Climber climb, double intakeAngleSup, double armAngleSup, double climbPosSup, BooleanSupplier ampBoolSup) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm, intake);
    this.arm = arm;
    this.intake = intake;
    this.climb = climb;
    this.climbPosSup = climbPosSup;
    this.intakeAngleSup = intakeAngleSup;
    this.armAngleSup = armAngleSup;
    this.ampBoolSup = ampBoolSup;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double intakeAngleVal = intakeAngleSup;
    double armAngleVal = armAngleSup;
    climb.encodedClimb(climbPosSup);
    if(ampBoolSup.getAsBoolean() == false) {
    arm.setAngle(armAngleVal);
    intake.setAngle(intakeAngleVal, armAngleVal);
    } else {
      arm.setAngle(-armAngleVal);
      intake.setAngle(-intakeAngleVal, -armAngleVal);
    }
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
