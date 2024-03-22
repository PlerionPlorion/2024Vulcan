// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class AutoShooter extends Command {
  private double shooterSpeedSup;
  private Intake shooter;
  private int counter = 0;
  private int target = 0;
  /** Creates a new Shooter. */
  public AutoShooter(Intake shooter, double shooterSpeedSup, double seconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.shooterSpeedSup = shooterSpeedSup;
    target = (int)( seconds * 50 );
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double shooterSpeedVal = shooterSpeedSup;
    if(counter < target) {
      counter++;
      shooter.shoot(shooterSpeedVal, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.shoot(0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return counter >= target;
  }
}
