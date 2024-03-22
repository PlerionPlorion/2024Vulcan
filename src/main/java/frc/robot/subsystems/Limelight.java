// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase {
  String limelightName;
  /** Creates a new Limelight. */
  public Limelight() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void gyroControl(boolean gyro) {
    if(gyro == true) {
      limelightName = "limelight-front";
    } else if (gyro == false){
      limelightName = "limelight-back";
    }
  }
    public double limelight_aim_proportional()
  {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    // double kP = .1;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double sign = LimelightHelpers.getTX(limelightName) >= 0 ? 1 : -1;
    double targetingAngularVelocity = LimelightHelpers.getBotPose3d_TargetSpace(limelightName).getRotation().getAngle() * sign;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= Constants.Swerve.maxAngularVelocity;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -0.03;
    if (Math.abs(LimelightHelpers.getBotPose3d_TargetSpace(limelightName).getRotation().getAngle()) >= 2) {
      SmartDashboard.putNumber("botPoseZ", LimelightHelpers.getBotPose3d_TargetSpace(limelightName).getRotation().getAngle());
    return targetingAngularVelocity;
    } else {
      SmartDashboard.putNumber("botPoseZ", LimelightHelpers.getBotPose3d_TargetSpace(limelightName).getRotation().getAngle());
      return 0;
    }
  }
  public double limelight_strafe_proportional()
  {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .1;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingStrafeVelocity = LimelightHelpers.getTX(limelightName) * kP;

    // convert to radians per second for our drive method
    targetingStrafeVelocity *= Constants.Swerve.maxSpeed;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingStrafeVelocity *= -0.5;
    if (Math.abs(LimelightHelpers.getTX(limelightName) * kP) >= 0.3) {
      SmartDashboard.putBoolean("limeRunStrafe", true);
    return targetingStrafeVelocity;
    } else {
      SmartDashboard.putBoolean("limeRunStrafe", false);
      return 0;
    }
  }
  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  public double limelight_range_proportional()
  {
    double kP = .1;
    double targetingForwardSpeed = LimelightHelpers.getTY(limelightName) * kP;
    targetingForwardSpeed *= Constants.Swerve.maxSpeed;
    targetingForwardSpeed *= -0.5;
    if (Math.abs(LimelightHelpers.getTY(limelightName)) >= 3) {
      SmartDashboard.putBoolean("limeRunForward", true);
      SmartDashboard.putNumber("TY", Math.abs(LimelightHelpers.getTX(limelightName)));
    return targetingForwardSpeed;
    } else {
      SmartDashboard.putBoolean("limeRunForward", false);
      return 0;
    }
  }

  public void limelightTagMode(boolean on) {
    if(on != true) {
      LimelightHelpers.setLEDMode_ForceOff("limelight-back");
      LimelightHelpers.setLEDMode_ForceOff("limelight-front");
      // LimelightHelpers.setPipelineIndex("limelight-front", 1);
    } else {
      LimelightHelpers.setLEDMode_ForceOn(limelightName);
      // LimelightHelpers.setPipelineIndex("limelight-front", 0);
    }
  }
}
