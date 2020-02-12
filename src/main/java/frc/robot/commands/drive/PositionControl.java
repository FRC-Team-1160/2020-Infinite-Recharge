/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import frc.robot.subsystems.Panel;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PositionControl extends CommandBase {
  //Creates a new PositionControl.
  private final Panel pan;
  
  public PositionControl(Panel pan) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pan);
    this.pan = pan;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pan.setTargetColor();
    pan.startSpinning(0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pan.stopSpinning();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pan.foundColor();
  }
}
