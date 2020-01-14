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
import com.revrobotics.EncoderType;
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
    
    private final int leftID = 0;
    private CANSparkMax leftMotor;
    private final int rightID = 1;
    private CANSparkMax rightMotor;

    private CANEncoder leftEncoder;
    private CANEncoder rightEncoder;

    private CANPIDController motorController;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

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
        leftMotor = new CANSparkMax(leftID, MotorType.kBrushed);                // leading spark
        leftEncoder = leftMotor.getEncoder(EncoderType.kQuadrature, 4096);

        rightMotor = new CANSparkMax(rightID, MotorType.kBrushed);
        rightEncoder = rightMotor.getEncoder(EncoderType.kQuadrature, 4096);

        setFollower(rightMotor, leftMotor);

        motorController = leftMotor.getPIDController();

        motorController.setFeedbackDevice(leftEncoder);

        // PID coefficients
        kP = 0.1; 
        kI = 1e-4;
        kD = 1; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = 1; 
        kMinOutput = -1;

        // set PID coefficients
        motorController.setP(kP);
        motorController.setI(kI);
        motorController.setD(kD);
        motorController.setIZone(kIz);
        motorController.setFF(kFF);
        motorController.setOutputRange(kMinOutput, kMaxOutput);
    }

    public void setFollower(CANSparkMax follower, CANSparkMax leader)
    {
        follower.follow(leader);
    }

    public void shootBottom(double input)  // shoots for bottom port (2 - 1pts)
    {
        // math
        shoot(input);
    }   

    public void shootOutter(double input)  // shoots for outter port (4 - 2pts)
    {
        // math
        shoot(input);
    }

    public void shootInner(double input)   // shoots for inner port (6 - 3pts)
    {
        // math
        shoot(input);
    }

    private void shoot(double input)   // enables PID
    {
        
    }

    public void set(double value)
    {
        
    }

    public void stop()
    {
        
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
}