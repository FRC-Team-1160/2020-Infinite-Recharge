/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class TurnToAngle extends PIDCommand {
  /**
   * Creates a new TurnToAngle.
   */
  public TurnToAngle(DriveTrain dt, double angleTarget) {
    super(
        // The controller that the command will use
        new PIDController(0.2, 0, 0),
        // This should return the measurement
        dt::getAngle,
        // This should return the setpoint (can also be a constant)
        angleTarget,
        // This uses the output
        output -> {}
        );
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().setSetpoint(angleTarget + dt.getAngle());
    getController().enableContinuousInput(-180, 180);
    getController().setTolerance(0.5);
    SmartDashboard.putData("Turn", getController());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
