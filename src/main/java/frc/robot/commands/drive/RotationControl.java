/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import frc.robot.subsystems.Panel;
import edu.wpi.first.wpilibj2.command.CommandBase;

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
    //the motor is reset and then it spins
    pan.startSpinning(1.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //stop the motor & reset it
    pan.stopSpinning();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //returns false until the color changes 26 times
    return pan.enoughSpins();
  }
}
