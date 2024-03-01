/*
 * Thanks NOMADs
 */

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase{
  private final CANSparkMax leftArmMotor = new CANSparkMax(12, MotorType.kBrushless);
  private final CANSparkMax rightArmMotor = new CANSparkMax(13, MotorType.kBrushless);
  private final CANcoder leftEncoder = new CANcoder(6, Constants.canBusName);
  //private final CANcoder rightEncoder = new CANcoder(5, Constants.canBusName);
  private TrapezoidProfile trapProfile;
  private State setpoint = new State();
  private State desiredState = new State();
  private double ffVolts;
  private SparkPIDController controllerLeft = leftArmMotor.getPIDController();
  private SparkPIDController controllerRight = rightArmMotor.getPIDController();
  public final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(
  Constants.ArmConstants.K_S, Constants.ArmConstants.K_V, Constants.ArmConstants.K_A);
  public Arm() {
    trapProfile = new TrapezoidProfile(Constants.ArmConstants.CONSTRAINTS);
    rightArmMotor.setInverted(true);
    controllerLeft.setP(0.001);
    controllerLeft.setI(0.0001);
    controllerLeft.setD(0.001);
    controllerLeft.setFF(0);
    controllerRight.setP(0.001);
    controllerRight.setI(0.0001);
    controllerRight.setD(0.001);
    controllerRight.setFF(0);
    leftArmMotor.setSmartCurrentLimit(30);
    rightArmMotor.setSmartCurrentLimit(30);
    leftEncoder.setPosition(leftEncoder.getAbsolutePosition().getValueAsDouble()); //The arm knows where it is
    // SmartDashboard.putNumber("GFF", 0.5);
    // SmartDashboard.putNumber("targetVelocity", 0);
    // SmartDashboard.putNumber("VFF", 0);
    // SmartDashboard.putNumber("K_S", 0.7);
  }
  public void periodic() {
    if(DriverStation.isEnabled()) {
      setpoint = trapProfile.calculate(0.02, setpoint, desiredState);

      if (Math.abs(Math.abs(desiredState.position) - Math.abs(leftEncoder.getPosition().getValueAsDouble()*2)) >= 0.2) { //It subtracts where it wants to go from where it is
      setpoint.position = -leftEncoder.getPosition().getValueAsDouble()*2; //The arm is where it is
      // SmartDashboard.putBoolean("running", true);
      } else {
        // SmartDashboard.putBoolean("running", false);
      }
      ffVolts = getGravityFF() + getVelocityFF(setpoint.velocity);
      //setPIDFF(setpoint.position, ffVolts);
      setVolts(ffVolts);
    } else {
      desiredState.velocity = 0;
      desiredState.position = -leftEncoder.getPosition().getValueAsDouble();
      setpoint.velocity = desiredState.velocity;
      setpoint.position = desiredState.position;
      setVolts(0);
    }
    //setVolts(getGravityFF() + Constants.ArmConstants.K_V * 5 + Constants.ArmConstants.K_S);
    // SmartDashboard.putNumber("ffVolts", ffVolts);
    // SmartDashboard.putNumber("Setpoint", setpoint.position);
    // SmartDashboard.putNumber("SetpointVelocity", setpoint.velocity);
    // SmartDashboard.putNumber("leftEncRelative", -leftEncoder.getPosition().getValueAsDouble()*2);
    // SmartDashboard.putNumber("DesiredState", desiredState.position);
    // SmartDashboard.putNumber("EncoderDiff", Math.abs(Math.abs(desiredState.position) - Math.abs(leftEncoder.getPosition().getValueAsDouble()*2)));
    // SmartDashboard.putNumber("leftEncoderVelocity", (-leftEncoder.getVelocity().getValueAsDouble()*90));
    // SmartDashboard.putNumber("VoltageLeft", leftArmMotor.getAppliedOutput());
    // SmartDashboard.putNumber("VoltageRight", rightArmMotor.getAppliedOutput());
  }

  public void setAngle(double angle) {
    desiredState.position = angle/45;
  }

  public Command runVoltage(DoubleSupplier voltage) {
    return run(()->setVolts(voltage.getAsDouble()));
  }

  public Command rotatetoAngle(DoubleSupplier angle) {
    return run(()->setAngle(angle.getAsDouble()));
  }

  public double getGravityFF() {
    //return SmartDashboard.getNumber("GFF", 0) * Math.sin(-leftEncoder.getPosition().getValueAsDouble()*360/Constants.ArmConstants.MOTOR_ROTATIONS_PER_ARM_ROTATION);
    return Constants.ArmConstants.K_G * Math.sin(-leftEncoder.getPosition().getValueAsDouble()*360/Constants.ArmConstants.MOTOR_ROTATIONS_PER_ARM_ROTATION);
  }

  public double getVelocityFF(double velocity) {
    return velocity * (Constants.ArmConstants.K_V) + (Math.signum(velocity) * Constants.ArmConstants.K_S);
  }

  public void setPIDFF(double angle, double ffVolts) {
    this.ffVolts = ffVolts;
    controllerLeft.setReference(angle, ControlType.kPosition, 0, ffVolts);
    controllerRight.setReference(angle, ControlType.kPosition, 0, ffVolts);
}

    public void setVolts(double volts) {
      rightArmMotor.setVoltage(volts);
      leftArmMotor.setVoltage(volts);
    }
}
