// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import static java.lang.Math.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.DrivePods.BLDP;
import frc.robot.DrivePods.BRDP;
import frc.robot.DrivePods.FLDP;
import frc.robot.DrivePods.FRDP;

public class Drivetrain extends CommandBase {
  private FLDP fldp;
  private FRDP frdp;
  private BLDP bldp;
  private BRDP brdp;

  private DoubleSupplier leftJoyX;
  private DoubleSupplier leftJoyY;
  private DoubleSupplier backJoyX;
  private DoubleSupplier backJoyY;

  public Drivetrain(FLDP fldp, FRDP frdp, BLDP bldp, BRDP brdp, DoubleSupplier leftJoyX, DoubleSupplier leftJoyY, DoubleSupplier backJoyX, DoubleSupplier backJoyY) {
    this.fldp = fldp;
    this.frdp = frdp;
    this.bldp = bldp;
    this.brdp = brdp;

    this.leftJoyX = leftJoyX;
    this.leftJoyY = leftJoyY;
    this.backJoyX = backJoyX;
    this.backJoyY = backJoyY;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

//coded with idea that 3 oclock is zero degrees, counterclockwise is positive, 2PI radians in a circle
//drivepod zero is facing forwards

    double targetHeading = atan2(leftJoyY.getAsDouble(), leftJoyX.getAsDouble());
    double error = gyro - targetHeading;
    double turnWheel = error * turnGain;




    double podTarget = atan2(bldpY, bldpX);
    double podAngle = bldp.getPodAngle();
    while (podAngle > 2*PI || podAngle < 0) {
      podAngle += (podAngle >= 2*PI) ? -2*PI : ((podAngle < 0) ? 2*PI : 0);
    }
    double podError = podTarget - podAngle;
    boolean backward = false;
    if (podError > PI/2) {
      podError += -PI;
      backward = true;
    } else if (podError < PI/-2) {
      podError += PI;
      backward = true;      
    }
    bldp.setAngle(podError + bldp.getPodAngle());
    bldp.setPower(sqrt(bldpX*bldpX+bldpY+bldpY) * backward ? -1 : 1);
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
