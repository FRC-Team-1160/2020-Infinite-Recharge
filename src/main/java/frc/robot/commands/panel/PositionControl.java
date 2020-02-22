/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.panel;

import frc.robot.subsystems.Panel;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.util.Color;

public class PositionControl extends CommandBase {
  //Creates a new PositionControl.
  private final Panel pan;
  private Color currentColor;
  
  public PositionControl(Panel pan) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pan);
    this.pan = pan;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pan.getPanelMotor().set(1.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pan.getPanelMotor().set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    currentColor = pan.getColorSensor().getColor();
    //to see if the current Color detected by the wheel is the color we want
    if(pan.getMatcher().matchClosestColor(pan.getTargetColor()).equals(pan.getMatcher().matchClosestColor(currentColor))){
      return true;
    }
    return false;
  }
}
