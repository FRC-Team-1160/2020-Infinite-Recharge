package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.DriveTrain;

public class AutoForwardShoot extends InstantCommand {

    public AutoForwardShoot(DriveTrain m_driveTrain, Delivery m_delivery) {
        super(new Runnable(){
        
            @Override
            public void run() {
                Timer m_timer = new Timer();

                m_timer.reset();
                m_timer.start();

                if (m_timer.get() < 2)
                {
                    m_driveTrain.tankDrive(-1.0, 1.0, 0.0);
                }else if (m_timer.get() < 13) {
                    m_delivery.shoot(1, 1, 1, 0);
                }

                if (m_timer.get() >= 15) return;
            }
        }, m_driveTrain, m_delivery);
	}
    
}
