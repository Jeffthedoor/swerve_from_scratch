package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DrivePods.BLDP;
import frc.robot.DrivePods.BRDP;
import frc.robot.DrivePods.FLDP;
import frc.robot.DrivePods.FRDP;
import frc.robot.commands.Drivetrain;

public class RobotContainer {

  private static final FLDP fldp = new FLDP();
  private static final FRDP frdp = new FRDP();
  private static final BLDP bldp = new BLDP();
  private static final BRDP brdp = new BRDP();


  private static final XboxController driver = new XboxController(0);

  public RobotContainer() {
    fldp.setDefaultCommand(new Drivetrain(fldp, frdp, bldp, brdp, () -> driver.getLeftX(), () -> driver.getLeftY(), () -> driver.getRightX(), () -> driver.getRightY()));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
