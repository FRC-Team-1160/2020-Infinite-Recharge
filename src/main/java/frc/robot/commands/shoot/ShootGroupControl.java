/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

public class ShootGroupControl extends CommandBase {
  /**
   * Creates a new ShooterControl.
   */
  private Shooter m_shooter;
  private double m_shootInput;
  private Feeder m_feeder;
  private double m_feedInput;
  private Timer timer;

  public ShootGroupControl(Shooter shooter, double m_shootInput, Feeder m_feeder, double m_feedInput) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_shooter = shooter;
    this.m_shootInput = m_shootInput;
    this.m_feeder = m_feeder;
    this.m_feedInput = m_feedInput;
    this.timer = new Timer();
    addRequirements(m_feeder, m_shooter);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_shooter.shooterControl(m_shootInput);
      if (timer.get() > 2.0) {
        /*
        new RunCommand(
                () -> m_feeder.feederControl(m_feedInput),
                m_feeder);*/
        m_feeder.feederControl(m_feedInput);
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.shooterControl(0);
    m_feeder.feederControl(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
