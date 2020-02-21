/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class TurnToAngle extends PIDCommand {
  /**
   * Creates a new TurnToAngle.
   */
  // new PIDCommand(m_driveTrain.turnController, m_driveTrain,       90.0f,        m_driveTrain,       m_driveTrain);
  public TurnToAngle(DriveTrain dt) {
    super(
        // The controller that the command will use
        dt.m_turnController,
        // This should return the measurement
        () -> dt.getYaw(),
        // This should return the setpoint (can also be a constant)
        () -> (dt.getYaw() - Vision.getTx()),
        // This uses the output
        output -> dt.accept(output)
        );
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  public TurnToAngle(DriveTrain dt, double setpoint) {
    super(
        // The controller that the command will use
        dt.m_turnController,
        // This should return the measurement
        () -> dt.getYaw(),
        // This should return the setpoint (can also be a constant)
        () -> (dt.getYaw() - setpoint),
        // This uses the output
        output -> dt.accept(output)
        );
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
