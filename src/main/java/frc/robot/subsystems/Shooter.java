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

        // PID coefficients
        kP = 0.1; 
        kI = 1e-4;
        kD = 1; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = 1; 
        kMinOutput = -1;

        // set PID coefficients
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
    }
}