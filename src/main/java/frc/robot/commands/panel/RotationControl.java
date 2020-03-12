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
  private final Panel m_panel;

  public RotationControl(Panel panel) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(panel);
    m_panel = panel;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_panel.getPanelEncoder().setPosition(0);
    m_panel.voltageSpin(3.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_panel.voltageSpin(3.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //stop the motor
    m_panel.voltageSpin(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //checks to see if the panelMotor has rotated enough times
    if(m_panel.getPanelEncoder().getPosition() > PanelConstants.MIN_REVS){
      return true;
    } 
    return false;
  }
}