/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.TurnToAngle;
import frc.robot.commands.shoot.ShootGroupControl;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShootSequence extends SequentialCommandGroup {
  /**
   * Creates a new ShootSequence.
   */
  public ShootSequence(DriveTrain m_driveTrain, Shooter m_shooter, Feeder m_feeder) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new TurnToAngle(m_driveTrain).withTimeout(5),
      new ShootGroupControl(m_shooter, -0.41 * 12, m_feeder, 3.6).withTimeout(5.0)
    );
  }
}
