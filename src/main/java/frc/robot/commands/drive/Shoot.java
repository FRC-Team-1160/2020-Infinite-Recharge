/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;


import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Shoot extends CommandBase {
  /**
   * Creates a new TurnToAngle.
   */

  private Shooter st;
  // new PIDCommand(m_driveTrain.turnController, m_driveTrain,       90.0f,        m_driveTrain,       m_driveTrain);
  public Shoot(Shooter st, double velocity) {
    this.st = st;
    st.run(velocity);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !st.status();
  }
}
