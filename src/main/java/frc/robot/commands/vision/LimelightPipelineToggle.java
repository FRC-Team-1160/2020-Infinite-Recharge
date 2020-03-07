/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.PipelineMode;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class LimelightPipelineToggle extends InstantCommand {
  public LimelightPipelineToggle() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(Vision.getValue("pipeline").getDouble(0.0) == PipelineMode.eLow.ordinal()){
      Vision.setPipeline(PipelineMode.eHigh.ordinal());
    }else{
      Vision.setPipeline(PipelineMode.eLow.ordinal());
    }
  }
}
