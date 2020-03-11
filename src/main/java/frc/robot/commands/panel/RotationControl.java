/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
 
package frc.robot.commands.panel;
 
import frc.robot.subsystems.Panel;
import frc.robot.Constants.PanelConstants;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
 
public class RotationControl extends CommandBase {
  // Creates a new Rotation.
  private Panel m_panel;
  private double m_input;
  private boolean m_volt;
  private Color currentColor;
  private int colorChangeCounter;
 
  public RotationControl(Panel panel, double input, boolean volt) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(panel);
    m_panel = panel;
    m_input = input;
    m_volt = volt;
    currentColor = m_panel.getColorSensor().getColor();
    colorChangeCounter = 0;
  }
 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_panel.getPanelEncoder().setPosition(0);
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
    //stop the motor
    if(m_volt){
      m_panel.voltageSpin(0);
    }else{
      m_panel.spin(0);
    }
  }
 
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Color currentColor = m_panel.getColorSensor().getColor();
    Color previousColor = m_panel.getColorSensor().getColor();
 
    //counts the number of times the color changes (the current color is different from the previous color)
    if(!m_panel.sameColor(currentColor, previousColor)){
      colorChangeCounter++;
    }
 
    if(colorChangeCounter >= 25){
      return true;
    } else {
      return false;
    }
  }
}
