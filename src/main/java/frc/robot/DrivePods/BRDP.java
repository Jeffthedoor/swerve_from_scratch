// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.DrivePods;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BRDP extends SubsystemBase {
  CANCoder canCoder;
  TalonFX steer;
  TalonFX drive;

  private ShuffleboardTab podTab = Shuffleboard.getTab("pods");
  private NetworkTableEntry podAngle = podTab.add("BR angle", 0).getEntry();
  private NetworkTableEntry driveOutput = podTab.add("BR output", 0).getEntry();

  public BRDP() {
    canCoder = new CANCoder(Constants.CANCODER_BACK_RIGHT);
    steer = new TalonFX(Constants.STEER_BACK_RIGHT);
    drive = new TalonFX(Constants.DRIVE_BACK_RIGHT);

    drive.setNeutralMode(NeutralMode.Coast);
    steer.setNeutralMode(NeutralMode.Coast);

    drive.setInverted(Constants.DRIVE_INVERT);
    steer.setInverted(Constants.STEER_INVERT);

    steer.setSelectedSensorPosition((canCoder.getAbsolutePosition() - Constants.CANCODER_OFFSET_BACK_RIGHT) * Constants.STEER_GEAR_RATIO * 2048);

    setGains();

    CommandScheduler.getInstance().registerSubsystem(this);
  }

  @Override
  public void periodic() {
    podAngle.setDouble(getPodAngle());
  }

  public double getPodAngle() {
      return steer.getSelectedSensorPosition();
      // return canCoder.getAbsolutePosition();
  }

  public void setPower(double power) {
    // drive.set(TalonFXControlMode.PercentOutput, power);
    driveOutput.setDouble(power);
  }

  public void setAngle(double angle) {
    steer.set(TalonFXControlMode.Position, angleToTicks(angle));
  }

  private double angleToTicks(double angle) {
    return angle * Constants.STEER_GEAR_RATIO * 2048 / 2 / Math.PI;
  }

  private void setGains() {
    steer.config_kP(0, Constants.STEER_P);
    steer.config_kI(0, Constants.STEER_I);
    steer.config_kD(0, Constants.STEER_D);
    steer.config_kF(0, Constants.STEER_F);

    steer.selectProfileSlot(0, 0);
  }
}
