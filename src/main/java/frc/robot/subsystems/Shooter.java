<<<<<<< Updated upstream
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
=======
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;

import frc.robot.Constants.*;

public class Shooter extends SubsystemBase
{
    private static Shooter instance;

    private final CANSparkMax   left,           right;
    private CANEncoder          leftEncoder,    rightEncoder;

    private CANPIDController speedController;

    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

    private double velocity;

    private boolean run;

    public static Shooter getInstance() {
        if (instance == null){
            instance = new Shooter();
        }
        return instance;
    }

    public Shooter() {
        left = new CANSparkMax(DriveConstants.LEFT_SHOOTER, MotorType.kBrushless);
        right = new CANSparkMax(DriveConstants.RIGHT_SHOOTER, MotorType.kBrushless);

        // inverse of left motor
        right.follow(left, true);

        leftEncoder = left.getEncoder();
        rightEncoder = right.getEncoder();

        // should be the same as right motor
        speedController = left.getPIDController();

        speedController.setFeedbackDevice(leftEncoder);
>>>>>>> Stashed changes

        // PID coefficients
        kP = 0.1; 
        kI = 1e-4;
        kD = 1; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = 1; 
        kMinOutput = -1;

        // set PID coefficients
<<<<<<< Updated upstream
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
=======
        speedController.setP(kP);
        speedController.setI(kI);
        speedController.setD(kD);
        speedController.setIZone(kIz);
        speedController.setFF(kFF);
        speedController.setOutputRange(kMinOutput, kMaxOutput);

        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput);
        SmartDashboard.putNumber("Set Velocity", 0);
    }

    public void run(double velocity)
    {
        this.velocity = velocity;
        if (velocity > 0) run = true;
        else run = false;
    }

    public boolean status()
    {
        return run;
    }

    public void periodic()
    {
        // read PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);
        velocity = SmartDashboard.getNumber("Set Velocity", 0);

        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((p != kP)) { speedController.setP(p); kP = p; }
        if((i != kI)) { speedController.setI(i); kI = i; }
        if((d != kD)) { speedController.setD(d); kD = d; }
        if((iz != kIz)) { speedController.setIZone(iz); kIz = iz; }
        if((ff != kFF)) { speedController.setFF(ff); kFF = ff; }
        if((max != kMaxOutput) || (min != kMinOutput)) { 
            speedController.setOutputRange(min, max); 
            kMinOutput = min; kMaxOutput = max; 
        }

        if (run)
        {
            speedController.setReference(velocity, ControlType.kVelocity);
        }

        SmartDashboard.putNumber("SetPoint", velocity);
        SmartDashboard.putNumber("ProcessVariable", leftEncoder.getVelocity());
>>>>>>> Stashed changes
    }
}