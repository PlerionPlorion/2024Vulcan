// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class TeleopIntake extends Command {
  private double intakeSpeedSup;
  private Intake intake;
  private int counter = 0;
  private int target = 0;
  /** Creates a new Shooter. */
  public TeleopIntake(Intake intake, double intakeSpeedSup, double seconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    this.intakeSpeedSup = intakeSpeedSup;
    this.intake = intake;
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
    double intakeSpeedVal = intakeSpeedSup;
    if(counter < target) {
      counter++;
      intake.intake(intakeSpeedVal);
    }
    SmartDashboard.putNumber("counter", counter);
  }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return counter >= target;
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.intake(0);
    SmartDashboard.putBoolean("intake", interrupted);

  }


}