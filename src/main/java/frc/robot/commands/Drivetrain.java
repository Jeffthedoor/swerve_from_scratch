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
    initialAngle = 0; // TODO: do gyro gyro.getAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

//coded with idea that 3 oclock is zero degrees, counterclockwise is positive, 2PI radians in a circle
//drivepod zero is facing forwards

    double currentAngle = 0; // TODO: do gyro (gyro.getAngle() - initialAngle) * PI / -180;

    double targetHeading = 0; // TODO: uncomment with do gyro /*atan(leftJoyX.getAsDouble()/leftJoyY.getAsDouble());*//*Math.*/atan2(leftJoyY.getAsDouble(), leftJoyX.getAsDouble());
    double robotTurnTarget = (currentAngle - targetHeading) * Constants.turnGain;

    double moveX = ((-1 * cos(currentAngle) * backJoyX.getAsDouble()) + (sin(currentAngle) * backJoyY.getAsDouble())) * Constants.moveGain;
    double moveY = ((-1 * cos(currentAngle) * backJoyY.getAsDouble()) + (sin(currentAngle) * backJoyX.getAsDouble())) * Constants.moveGain;




    double bldpX = cos(PI*5/4)*robotTurnTarget + moveX;
    double bldpY = sin(PI*5/4)*robotTurnTarget + moveY;

    double blpodMove = atan2(bldpY, bldpX);
    double blpodAngle = bldp.getPodAngle();
    
    blpodAngle = blpodAngle % 2*PI;
    blpodAngle += (blpodAngle >= PI) ? -PI : (blpodAngle <= PI) ? PI : 0;
    double blpodError = blpodMove - blpodAngle;
    blpodError += (blpodError >= PI) ? -PI : (blpodError <= PI) ? PI : 0;

    boolean blbackward = false;
    if (blpodError > PI/2) {
      blpodError += -PI;
      blbackward = true;
    } else if (blpodError < PI/-2) {
      blpodError += PI;
      blbackward = true;      
    }
    bldp.setAngle(blpodError + blpodAngle);
    bldp.setPower(sqrt(bldpX*bldpX+bldpY*bldpY) * (blbackward ? -1 : 1));


    double brdpX = cos(PI*3/4)*robotTurnTarget + moveX;
    double brdpY = sin(PI*3/4)*robotTurnTarget + moveY;

    double brpodMove = atan2(brdpY, brdpX);
    double brpodAngle = brdp.getPodAngle();

    brpodAngle = brpodAngle % 2*PI;
    brpodAngle += (brpodAngle >= PI) ? -PI : (brpodAngle <= PI) ? PI : 0;
    double brpodError = brpodMove - brpodAngle;
    brpodError += (brpodError >= PI) ? -PI : (brpodError <= PI) ? PI : 0;    
    boolean brbackward = false;

    if (brpodError > PI/2) {
      brpodError -= PI;
      brbackward = true;
    } else if (brpodError < PI/-2) {
      brpodError += PI;
      brbackward = true;      
    }
    brdp.setAngle(brpodError + brpodAngle);
    brdp.setPower(sqrt(brdpX*brdpX+brdpY*brdpY) * (brbackward ? -1 : 1));


    double frdpX = cos(PI*1/4)*robotTurnTarget + moveX;
    double frdpY = sin(PI*1/4)*robotTurnTarget + moveY;

    double frpodMove = atan2(frdpY, frdpX);
    double frpodAngle = frdp.getPodAngle();

    frpodAngle = frpodAngle % 2*PI;
    frpodAngle += (frpodAngle >= PI) ? -PI : (frpodAngle <= PI) ? PI : 0;
    double frpodError = frpodMove - frpodAngle;
    frpodError += (frpodError >= PI) ? -PI : (frpodError <= PI) ? PI : 0;

    boolean frbackward = false;
    if (frpodError > PI/2) {
      frpodError += -PI;
      frbackward = true;
    } else if (frpodError < PI/-2) {
      frpodError += PI;
      frbackward = true;      
    }
    frdp.setAngle(frpodError + frpodAngle);
    frdp.setPower(sqrt(frdpX*frdpX+frdpY*frdpY) * (frbackward ? -1 : 1));


    double fldpX = cos(PI*7/4)*robotTurnTarget + moveX;
    double fldpY = sin(PI*7/4)*robotTurnTarget + moveY;

    double flpodMove = atan2(fldpY, fldpX);
    double flpodAngle = fldp.getPodAngle();

    flpodAngle = flpodAngle % 2*PI;
    flpodAngle += (flpodAngle >= PI) ? -PI : (flpodAngle <= PI) ? PI : 0;
    double flpodError = flpodMove - flpodAngle;
    flpodError += (flpodError >= PI) ? -PI : (flpodError <= PI) ? PI : 0;

    boolean flbackward = false;
    if (flpodError > PI/2) {
      flpodError -= PI;
      flbackward = true;
    } else if (flpodError < PI/-2) {
      flpodError += PI;
      flbackward = true;      
    }
    fldp.setAngle(flpodError + flpodAngle);
    fldp.setPower(sqrt(fldpX*fldpX+fldpY*fldpY) * (flbackward ? -1 : 1));

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
