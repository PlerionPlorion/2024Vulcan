// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class TeleopLimelightDrive extends Command {
  Swerve swerve;
  Limelight limelight;
  IntakePivot intake;
  Arm arm;
  boolean amp;
  ChassisSpeeds relativeSpeed;
  boolean gyro;
  int invert;
  /** Creates a new TeleopLimelightDrive. */
  public TeleopLimelightDrive(Swerve swerve, Limelight limelight, IntakePivot intake, Arm arm, boolean amp) {
    this.swerve = swerve;
    this.limelight = limelight;
    this.intake = intake;
    this.arm = arm;
    this.amp = amp;
    addRequirements(swerve, limelight, intake, arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(swerve.getHeading().getDegrees() >= -90 && swerve.getHeading().getDegrees() <= 90){
      gyro = false;
      invert = -1;
    } else {
      gyro = true;
      invert = 1;
    }
    limelight.gyroControl(gyro);
    limelight.limelightTagMode(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        if(swerve.getHeading().getDegrees() >= -90 && swerve.getHeading().getDegrees() <= 90){
          gyro = false;
          invert = -1;
        } else {
          gyro = true;
          invert = 1;
        }
        limelight.gyroControl(gyro);
        SmartDashboard.putBoolean("gyroBool", gyro);
            /* Get Values, Deadband*/
        double translationVal = limelight.limelight_range_proportional() * invert;
        double strafeVal = limelight.limelight_strafe_proportional() * invert;
        double rotationVal = limelight.limelight_aim_proportional();
        relativeSpeed = new ChassisSpeeds(translationVal, strafeVal, 0);
        /* Drive */
        swerve.driveRobotRelative(relativeSpeed);
        if(amp != true) {
          intake.setAngle((37)*invert, 0);
          arm.setAngle((-10)*invert);
        }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    limelight.limelightTagMode(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
