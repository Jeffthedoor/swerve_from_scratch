// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.kauailabs.navx.frc.AHRS;

import static java.lang.Math.*;

import edu.wpi.first.wpilibj.I2C.Port;
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

  private double initialAngle;

  private AHRS gyro = new AHRS(Port.kMXP);
  //WPI_PigeonIMU gyro = new WPI_PigeonIMU(0); // Pigeon is on CAN Bus with device ID 0

  public Drivetrain(FLDP fldp, FRDP frdp, BLDP bldp, BRDP brdp, DoubleSupplier leftJoyX, DoubleSupplier leftJoyY, DoubleSupplier backJoyX, DoubleSupplier backJoyY) {
    this.fldp = fldp;
    this.frdp = frdp;
    this.bldp = bldp;
    this.brdp = brdp;

    this.leftJoyX = leftJoyX;
    this.leftJoyY = leftJoyY;
    this.backJoyX = backJoyX;
    this.backJoyY = backJoyY;

    addRequirements(fldp, frdp, bldp, brdp);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialAngle = gyro.getAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

//coded with idea that 3 oclock is zero degrees, counterclockwise is positive, 2PI radians in a circle
//drivepod zero is facing forwards

    double currentAngle = (gyro.getAngle() - initialAngle) * PI / -180;

    double targetHeading = /*atan(leftJoyX.getAsDouble()/leftJoyY.getAsDouble());*//*Math.*/atan2(leftJoyY.getAsDouble(), leftJoyX.getAsDouble());
    // double error = gyro - targetHeading;
    double robotTurnTarget = (currentAngle - targetHeading) * Constants.turnGain;

    double moveX = cos(currentAngle) * backJoyX.getAsDouble() * Constants.moveGain;
    double moveY = sin(currentAngle) * backJoyY.getAsDouble() * Constants.moveGain;




    double bldpX = cos(PI*5/4)*robotTurnTarget + moveX;
    double bldpY = sin(PI*5/4)*robotTurnTarget + moveY;

    double podMove = atan2(bldpY, bldpX);
    double podAngle = bldp.getPodAngle();
    while (podAngle > 2*PI || podAngle <= 0) {
      podAngle += (podAngle >= 2*PI) ? -2*PI : ((podAngle < 0) ? 2*PI : 0);
    }
    double podError = podMove - podAngle;
    boolean backward = false;
    if (podError > PI/2) {
      podError += -PI;
      backward = true;
    } else if (podError < PI/-2) {
      podError += PI;
      backward = true;      
    }
    bldp.setAngle(podError + bldp.getPodAngle());
    bldp.setPower(sqrt(bldpX*bldpX+bldpY*bldpY) * (backward ? -1 : 1));


    double brdpX = cos(PI*3/4)*robotTurnTarget + moveX;
    double brdpY = sin(PI*3/4)*robotTurnTarget + moveY;

    podMove = atan2(brdpY, brdpX);
    podAngle = brdp.getPodAngle();
    while (podAngle > 2*PI || podAngle <= 0) {
      podAngle += (podAngle >= 2*PI) ? -2*PI : ((podAngle < 0) ? 2*PI : 0);
    }
    podError = podMove - podAngle;
    backward = false;
    if (podError > PI/2) {
      podError += -PI;
      backward = true;
    } else if (podError < PI/-2) {
      podError += PI;
      backward = true;      
    }
    brdp.setAngle(podError + brdp.getPodAngle());
    brdp.setPower(sqrt(brdpX*brdpX+brdpY*brdpY) * (backward ? -1 : 1));


    double frdpX = cos(PI*1/4)*robotTurnTarget + moveX;
    double frdpY = sin(PI*1/4)*robotTurnTarget + moveY;

    podMove = atan2(frdpY, frdpX);
    podAngle = frdp.getPodAngle();
    while (podAngle > 2*PI || podAngle <= 0) {
      podAngle += (podAngle >= 2*PI) ? -2*PI : ((podAngle < 0) ? 2*PI : 0);
    }
    podError = podMove - podAngle;
    backward = false;
    if (podError > PI/2) {
      podError += -PI;
      backward = true;
    } else if (podError < PI/-2) {
      podError += PI;
      backward = true;      
    }
    frdp.setAngle(podError + frdp.getPodAngle());
    frdp.setPower(sqrt(frdpX*frdpX+frdpY*frdpY) * (backward ? -1 : 1));


    double fldpX = cos(PI*1/4)*robotTurnTarget + moveX;
    double fldpY = sin(PI*1/4)*robotTurnTarget + moveY;

    podMove = atan2(fldpY, fldpX);
    podAngle = fldp.getPodAngle();
    while (podAngle > PI || podAngle <= -PI) {
      podAngle += (podAngle >= PI) ? -2*PI : ((podAngle < -PI) ? 2*PI : 0);
    }
    podError = podMove - podAngle;
    backward = false;
    if (podError > PI/2) {
      podError -= PI;
      backward = true;
    } else if (podError < PI/-2) {
      podError += PI;
      backward = true;      
    }
    fldp.setAngle(podError + fldp.getPodAngle());
    fldp.setPower(sqrt(fldpX*fldpX+fldpY*fldpY) * (backward ? -1 : 1));

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
