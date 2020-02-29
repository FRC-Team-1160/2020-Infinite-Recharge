/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class PIDShooterControl extends CommandBase {
  /**
   * Creates a new PIDShooterControl.
   */
  private Shooter m_shooter;
  private double m_input;

  public PIDShooterControl(Shooter shooter, double input) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_input = input;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.PIDShooterPositionControl(m_input);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.PIDShooterPositionControl(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
