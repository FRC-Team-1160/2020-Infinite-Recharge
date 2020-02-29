/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.panel;

import frc.robot.subsystems.Panel;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PanelConstants;

public class RotationControl extends CommandBase {
  // Creates a new Rotation.
  private final Panel pan;

  public RotationControl(Panel pan) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pan);
    this.pan = pan;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pan.getPanelEncoder().setPosition(0);
    pan.spin(1.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //stop the motor
    pan.spin(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //checks to see if the panelMotor has rotated enough times
    if(pan.getPanelEncoder().getPosition() > PanelConstants.MIN_REVS){
      return true;
    } else {
      return false;
    }
  }
}