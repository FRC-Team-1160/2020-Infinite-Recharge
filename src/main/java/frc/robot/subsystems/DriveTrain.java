/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.PortConstantsFinal;
import frc.robot.commands.drive.TurnToAngle;

public class DriveTrain extends SubsystemBase{
  /** 
   * Creates a new DriveTrain.
   */
  private static DriveTrain m_instance;

  private final CANSparkMax m_frontLeft, m_middleLeft, m_backLeft, m_frontRight, m_middleRight, m_backRight;
  private CANEncoder m_leftEncoder, m_rightEncoder;
  private CANPIDController m_leftController, m_rightController;

  private DifferentialDrive m_drive;

  private final DifferentialDriveOdometry m_odometry;
  public AHRS m_gyro;

  public PIDController m_turnController;

  private double straightAngle; 
  private boolean straightAngleSet;

  private Timer m_timer;

  private double grandKP;

  public final DifferentialDriveKinematics kDriveKinematics;
  // private Compressor m_comp;

  public static DriveTrain getInstance(){
    if (m_instance == null){
      m_instance = new DriveTrain();
    }
    return m_instance;
  }

  public DriveTrain() {

    grandKP = DriveConstants.kP_DRIVE;

    if (Constants.isFinal){
      m_frontLeft = new CANSparkMax(PortConstantsFinal.FRONT_LEFT_DRIVE, MotorType.kBrushless);
      m_middleLeft = new CANSparkMax(PortConstantsFinal.MIDDLE_LEFT_DRIVE, MotorType.kBrushless);
      m_backLeft = new CANSparkMax(PortConstantsFinal.BACK_LEFT_DRIVE, MotorType.kBrushless); 
      
      m_frontRight = new CANSparkMax(PortConstantsFinal.FRONT_RIGHT_DRIVE, MotorType.kBrushless);
      m_middleRight = new CANSparkMax(PortConstantsFinal.MIDDLE_RIGHT_DRIVE, MotorType.kBrushless);
      m_backRight = new CANSparkMax(PortConstantsFinal.BACK_RIGHT_DRIVE, MotorType.kBrushless);  

    } else {
      m_frontLeft = new CANSparkMax(PortConstants.FRONT_LEFT_DRIVE, MotorType.kBrushless);
      m_middleLeft = new CANSparkMax(PortConstants.MIDDLE_LEFT_DRIVE, MotorType.kBrushless);
      m_backLeft = new CANSparkMax(PortConstants.BACK_LEFT_DRIVE, MotorType.kBrushless); 
      
      m_frontRight = new CANSparkMax(PortConstants.FRONT_RIGHT_DRIVE, MotorType.kBrushless);
      m_middleRight = new CANSparkMax(PortConstants.MIDDLE_RIGHT_DRIVE, MotorType.kBrushless);
      m_backRight = new CANSparkMax(PortConstants.BACK_RIGHT_DRIVE, MotorType.kBrushless);  
    }
    
    m_frontLeft.restoreFactoryDefaults();
    m_middleLeft.restoreFactoryDefaults();
    m_backLeft.restoreFactoryDefaults();

    m_frontRight.restoreFactoryDefaults();
    m_middleRight.restoreFactoryDefaults();
    m_backRight.restoreFactoryDefaults();

    m_frontLeft.follow(m_backLeft);
    m_middleLeft.follow(m_backLeft);
    
    m_frontRight.follow(m_backRight);
    m_middleRight.follow(m_backRight);

    m_leftEncoder = m_backLeft.getEncoder();
    m_rightEncoder = m_backRight.getEncoder();

    // m_leftEncoder.setPositionConversionFactor(factor);

    // important
    // m_leftEncoder.setVelocityConversionFactor(factor);

    m_leftController = m_backLeft.getPIDController();
    m_rightController = m_backRight.getPIDController();

    m_drive = new DifferentialDrive(m_backLeft, m_backRight);

    m_gyro = new AHRS(Port.kMXP);
 
    m_turnController = new PIDController(AutoConstants.TURN_KP, AutoConstants.TURN_KI, AutoConstants.TURN_KD);
    m_turnController.enableContinuousInput(AutoConstants.MIN_INPUT, AutoConstants.MAX_INPUT);
    m_turnController.setIntegratorRange(AutoConstants.MIN_INGL, AutoConstants.MAX_INGL);
    m_turnController.setTolerance(AutoConstants.TOLERANCE);
    m_turnController.setSetpoint(0);

    SmartDashboard.putData("Turn Controller", m_turnController);

    straightAngle = 0.0;
    straightAngleSet = false;

    m_timer = new Timer();

    kDriveKinematics = new DifferentialDriveKinematics(AutoConstants.kTRACKWIDTH);

    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getYaw()));

    // SmartDashboard.putNumber("TURN_KP", AutoConstants.TURN_KP);
    //SmartDashboard.putNumber("kS", AutoConstants.kS);
    SmartDashboard.putNumber("KP", grandKP);

  }

  public void voltageDrive(double voltage) // moves each gearbox accordingly
  {
    double sign = Math.signum(voltage);
    m_backLeft.setVoltage(sign*AutoConstants.kS_CONCRETE + voltage);
    m_backRight.setVoltage(sign*AutoConstants.kS_CONCRETE + voltage);
    // SmartDashboard.putNumber("voltage in turn", voltage);
  }
  
  public void tankDrive(double x, double z, double lowLeft, double lowRight){

    double adaptedZ = 0;

    /*
    if(Math.abs(z) < DriveConstants.ANGLE_THRESHOLD && Math.abs(lowLeft) < DriveConstants.ANGLE_THRESHOLD && Math.abs(lowRight) < DriveConstants.ANGLE_THRESHOLD){
      if(!(m_gyro.isMoving() || straightAngleSet)){
        straightAngle = m_gyro.getAngle();
        straightAngleSet = true;
        SmartDashboard.putNumber("M", 1);
      } else if (straightAngleSet){
          // adaptedZ = DriveConstants.kP_DRIVE*(straightAngle-m_gyro.getAngle());
          SmartDashboard.putNumber("M", 2);
          if(x < 0){
            // adaptedZ = grandKP*(straightAngle-m_gyro.getAngle());
            adaptedZ = 0;
          }else{
            // adaptedZ = grandKP*(straightAngle-m_gyro.getAngle());
            adaptedZ = 0;
          }

      }
    }else{
      SmartDashboard.putNumber("M", 3);
      straightAngleSet = false;
      if(lowLeft > 0 || lowRight > 0){
        adaptedZ = DriveConstants.TURN_FACTOR*DriveConstants.LOW_DPI*(lowRight-lowLeft);
      } else {
        adaptedZ = DriveConstants.TURN_FACTOR*z;
      }
    }
    */
    
    straightAngleSet = false;
    if(lowLeft > 0 || lowRight > 0){
      adaptedZ = DriveConstants.TURN_FACTOR*DriveConstants.LOW_DPI*(lowRight-lowLeft);
    } else {
      adaptedZ = DriveConstants.TURN_FACTOR*z;
    }

    SmartDashboard.putNumber("Z", adaptedZ);

    SmartDashboard.putNumber("straightAngle", straightAngle);

    SmartDashboard.putNumber("currentAngle", m_gyro.getAngle());

    SmartDashboard.putNumber("put angle diff", (straightAngle-m_gyro.getAngle()));

    SmartDashboard.putBoolean("moving", m_gyro.isMoving());
  
    double[] outputs = scale(x, adaptedZ);
    m_backLeft.setVoltage(outputs[0]);
    m_backRight.setVoltage(outputs[1]);

    m_drive.feed();
  }

  public double[] scale(double x, double z){    
    double rawLeftInput = -x+z;
    double rawRightInput = x+z;

    // forward: left pos, right neg
    double leftDirection = Math.signum(rawLeftInput);
    double rightDirection = Math.signum(rawRightInput);

    double leftOutput = AutoConstants.kS*leftDirection + AutoConstants.kV*(rawLeftInput);
    double rightOutput = AutoConstants.kS*rightDirection + AutoConstants.kV*(rawRightInput);

    double[] rawInputs = {rawLeftInput, rawRightInput};
    double[] directions = {leftDirection, rightDirection};
    double[] outputs = {leftOutput, rightOutput};

    SmartDashboard.putNumberArray("inputs", rawInputs);
    SmartDashboard.putNumberArray("directions", directions);
    SmartDashboard.putNumberArray("outputs", outputs);

    // when you run the SPARK MAX in voltage mode there is no control loop, it is still running open loop
    // https://www.chiefdelphi.com/t/frc-characterization-output-driven-results/374592/9
    // m_leftController.setReference(leftOutput, ControlType.kVoltage);
    // m_rightController.setReference(rightOutput, ControlType.kVoltage);
    
    double finalOutputLeft = cubic(leftOutput);
    double finalOutputRight = cubic(rightOutput);
    
    double[] finalOutputs = {finalOutputLeft, finalOutputRight};

    SmartDashboard.putNumberArray("finaloutputs", finalOutputs);

    return finalOutputs;
  }

  public double cubic(double input){
    return 0.5*Math.pow(input, 3) + 0.5*Math.pow(input, 1);
  }

    /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
  }

  /**
   * Resets the Yaw of the Gyro.
   *
   */

  public void resetYaw(){
    m_gyro.reset();
  }

  /**
   * Returns the yaw of the Gyro.
   *
   * @return The yaw.
   */
  public double getYaw() {
    return m_gyro.getYaw();
  }

  /**
   * Resets the yaw of the Gyro.
   *
   */
  public void zeroYaw() {
    m_gyro.reset();
  }

    /**
   * Returns the pitch of the Gyro.
   *
   * @return The pitch.
   */
  public double getPitch(){
    return m_gyro.getPitch();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  
  @Override
  public void periodic() {

    /*
    double[] outputs = {m_frontLeft.getBusVoltage(), 
      m_middleLeft.getBusVoltage(),
      m_backLeft.getBusVoltage(),
      m_frontRight.getBusVoltage(),
      m_middleRight.getBusVoltage(),
      m_backRight.getBusVoltage(),
    };
    */

    // SmartDashboard.putNumberArray("outputs", outputs);
    // AutoConstants.TURN_KP = SmartDashboard.getNumber("TURN_KP", 0.0);
    // AutoConstants.kS = SmartDashboard.getNumber("kS", 0.0);
    // m_turnController.setP(AutoConstants.TURN_KP);

    SmartDashboard.putNumber("KP CHECK", grandKP);

    grandKP = SmartDashboard.getNumber("KP", 0.0);

    m_odometry.update(Rotation2d.fromDegrees(getYaw()), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
  }
}
