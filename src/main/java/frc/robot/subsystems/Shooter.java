/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.command.Subsystem;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Shooter extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    private static Shooter instance;
    private WPI_TalonSRX wheel;

    public final PIDController shootController;

    double kP = 0;
    double kI = 0;
    double kD = 0;
    double minimumInput;
    double maximumInput;
    double minimumOutput;
    double maximumOutput;
    double positionTolerance;

    public static Shooter GetInstance()
    {
        if (instance == null)
        {
            instance = new Shooter();
        }

        return instance;
    }

    private Shooter()
    {
        wheel = new WPI_TalonSRX(SHOOTER_WHEEL1);
        
        minimumInput = 0.0f;
        maximumInput = 5.0f;
        minimumOutput = -0.45f;
        maximumOutput = 0.45f;
        positionTolerance = 0.1f;
        shootController = new PIDController(kP, kI, kD);
        shootController.setIntegratorRange(minimumOutput, maximumOutput);
        shootController.setTolerance(positionTolerance);
    }

    public void shootBottom(double dist, double input)  // shoots for bottom port (2 - 1pts)
    {
        shoot(dist, input);
    }   

    public void shootOutter(double dist, double input)  // shoots for outter port (4 - 2pts)
    {
        shoot(dist, input);
    }

    public void shootInner(double dist, double input)   // shoots for inner port (6 - 3pts)
    {
        shoot(dist, input);
    }

    private void shoot(double dist, double input)   // enables PID
    {
        shootController.reset();
        shootController.setPID(kP, kI, kD);
        shootController.enableContinuousInput(minimumInput, maximumInput);
        while (shootController.atSetpoint() == false)
        {
            // calculate returns output                                  measurement
            wheel.set(shootController.calculate(input));
        }
        shootController.disableContinuousInput();
    }

    public void set(double value)
    {
        
    }

    public void stop()
    {
        wheel.set(0);
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
}