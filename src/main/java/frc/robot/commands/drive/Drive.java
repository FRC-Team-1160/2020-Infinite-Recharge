package frc.robot.commands.drive;

import java.util.Set;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.shoot.ShooterControl;
import frc.robot.subsystems.DriveTrain;

public class Drive extends CommandBase {
    private final DriveTrain m_driveTrain;

    public Drive(DriveTrain m_driveTrain) {
        this.m_driveTrain = m_driveTrain;
        super.setName("Drive");
        super.addRequirements(this.m_driveTrain);
    }
    
    @Override
    public void execute() {
        m_driveTrain.tankDrive(0.5, 0.0, 0.0, 0.0);
    }
}
