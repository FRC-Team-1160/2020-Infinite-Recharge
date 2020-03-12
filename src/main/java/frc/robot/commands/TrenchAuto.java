/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Vision.LightMode;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class TrenchAuto extends SequentialCommandGroup {
  /**
   * Creates a new TrenchAuto.
   */
  public TrenchAuto(DriveTrain m_driveTrain, Shooter m_shooter, Feeder m_feeder) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new RunCommand(() -> Vision.setLedMode(LightMode.eOn)),
      new ShootSequence(m_driveTrain, m_shooter, m_feeder),
      new TurnToAngle(m_driveTrain, m_driveTrain.getLastTurnNumber()),
      new Drive(m_driveTrain, 1).withTimeout(4),
      new ShootSequence(m_driveTrain, m_shooter, m_feeder),
      new TurnToAngle(m_driveTrain, m_driveTrain.getLastTurnNumber()),
      new RunCommand(() -> Vision.setLedMode(LightMode.eOff))
    );
  }
}
