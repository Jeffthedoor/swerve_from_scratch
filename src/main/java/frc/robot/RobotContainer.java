package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

  public RobotContainer() {
    configureButtonBindings();
    setDefaultCommands();
  }

  private void configureButtonBindings() {}

  private void setDefaultCommands() {

  }

  public Command getAutonomousCommand() {
    return null;
  }
}
