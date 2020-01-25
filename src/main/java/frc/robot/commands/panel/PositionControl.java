/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.panel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Panel;

public class PositionControl extends CommandBase {
  /**
   * Creates a new PositionControl.
   */
  private char curColor;

  private Panel m_pn;

  public PositionControl(Panel pn, String target) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_pn = pn;
    addRequirements(m_pn);
    curColor = ' ';
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    curColor = m_pn.getTarget();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_pn.setSpinner(AutoConstants.POSITION_SPEED);
    if(curColor != m_pn.getColor()){
      curColor = m_pn.getColor();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pn.setSpinner(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_pn.getTarget() == 'N') || (m_pn.getTarget() == curColor);
  }
}
