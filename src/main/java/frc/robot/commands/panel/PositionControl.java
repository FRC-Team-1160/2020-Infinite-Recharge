/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
 
package frc.robot.commands.panel;
 
import frc.robot.subsystems.Panel;
import edu.wpi.first.wpilibj2.command.CommandBase;
 
public class PositionControl extends CommandBase {
  //Creates a new PositionControl.
  private Panel m_panel;
  private double m_input;
  private boolean m_volt;
  
  public PositionControl(Panel panel, double input, boolean volt) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_panel);
    m_panel = panel;
    m_input = input;
    m_volt = volt;
  }
 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }
 
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_volt){
      m_panel.voltageSpin(m_input);
    }else{
      m_panel.spin(m_input);
    }
  }
 
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(m_volt){
      m_panel.voltageSpin(0);
    }else{
      m_panel.spin(0);
    }
  }
 
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //checks to see if the current color is the same as the target color
    return m_panel.sameColor();
  }
}
