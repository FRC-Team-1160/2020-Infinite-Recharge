package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutoConstantsFinal;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.climb.ClimbControl;
import frc.robot.commands.drive.Drive;
import frc.robot.commands.drive.TurnToAngle;
import frc.robot.commands.feed.FeederControl;
import frc.robot.commands.intake.*;
import frc.robot.commands.panel.*;
import frc.robot.commands.shoot.*;
import frc.robot.commands.vision.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    public final DriveTrain m_driveTrain = DriveTrain.getInstance(); 

    public final Intake m_intake = Intake.getInstance();

    public final Feeder m_feeder = Feeder.getInstance();

    public final Shooter m_shooter = Shooter.getInstance();

    public final Panel m_panel = Panel.getInstance();

    public final Climber m_climber = Climber.getInstance();

    // The autonomous routines
  
    // A simple auto routine that drives forward a specified distance, and then stops.
    //private final Command m_simpleAuto =
        //new DriveDistance(AutoConstants.kAutoDriveDistanceInches, AutoConstants.kAutoDriveSpeed,
                         // m_robotDrive);
  
    private final Command m_turnRight =
      new TurnToAngle(m_driveTrain, 45);

    private final Command m_turnLeft =
      new TurnToAngle(m_driveTrain, -45);
    // A complex auto routine that drives forward, drops a hatch, and then drives backward.
    //private final Command m_complexAuto = new ComplexAuto(m_robotDrive, m_hatchSubsystem);
    private final Command m_shootBack =
    new SequentialCommandGroup(
      //  new ParallelCommandGroup(
      //    new ShooterControl(m_shooter, -0.9),
      //    new RunCommand(
      //       () -> m_feeder.feederControl(0.3),
      //       m_feeder)
      //  ).withTimeout(5.0),
      new ShootGroupControl(m_shooter, -0.41 * 12, m_feeder, 0.3 * 12).withTimeout(5.0),
      new Drive(m_driveTrain, 0.5).withTimeout(2) // no * 12 because it is still .set, not .setVoltage
    );

    private final Command m_turnShootBack = 
      // new ShootGroupControl(m_shooter, -0.9, m_feeder, 0.7);
      new SequentialCommandGroup(
        //  new ParallelCommandGroup(
        //    new ShooterControl(m_shooter, -0.9),
        //    new RunCommand(
        //       () -> m_feeder.feederControl(0.3),
        //       m_feeder)
        //  ).withTimeout(5.0),
        new TurnToAngle(m_driveTrain).withTimeout(5),
        new ShootGroupControl(m_shooter, -0.41 * 12, m_feeder, 0.3 * 12).withTimeout(5.0),
        new Drive(m_driveTrain, 0.5).withTimeout(2) // no * 12 because it is still .set, not .setVoltage
      );

    private final Command m_turnShootBackMore = 
      // new ShootGroupControl(m_shooter, -0.9, m_feeder, 0.7);
      new SequentialCommandGroup(
        //  new ParallelCommandGroup(
        //    new ShooterControl(m_shooter, -0.9),
        //    new RunCommand(
        //       () -> m_feeder.feederControl(0.3),
        //       m_feeder)
        //  ).withTimeout(5.0),
        new TurnToAngle(m_driveTrain).withTimeout(5),
        new ShootGroupControl(m_shooter, -0.41 * 12, m_feeder, 0.3 * 12).withTimeout(5.0),
        new Drive(m_driveTrain, 0.5).withTimeout(3) // no * 12 because it is still .set, not .setVoltage
      );

    // private final Command m_back = new Drive(m_driveTrain, 0.5).withTimeout(2); // no * 12 because it is still .set, not .setVoltage

    // A chooser for autonomous commands
    SendableChooser<Command> m_chooser = new SendableChooser<>();
  
    // The driver's controller
    private Joystick m_mainStick = new Joystick(OIConstants.mainStickPort);
  
    // Secondary controllers
    private Joystick m_firstStick = new Joystick(OIConstants.firstStickPort);

    private Joystick m_secondStick = new Joystick(OIConstants.secondStickPort);


    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

      Vision.setPipeline(1);

      // Configure the button bindings
      configureButtonBindings();
  
      // Configure default commands
      
      m_driveTrain.setDefaultCommand(new RunCommand(
        () -> m_driveTrain.tankDrive(m_mainStick.getRawAxis(1), m_mainStick.getRawAxis(4), m_mainStick.getRawAxis(2), m_mainStick.getRawAxis(3)),
        m_driveTrain)
      );

      m_feeder.setDefaultCommand(new RunCommand(
        () -> m_feeder.feederControl(m_secondStick.getRawAxis(1) * 11),
        m_feeder)
      );

  
      // Add commands to the autonomous command chooser
      // m_chooser.addOption("Simple Auto", m_simpleAuto);
      // m_chooser.addOption("Complex Auto", m_complexAuto);

      m_chooser.addOption("Turn Right", m_turnRight);
      m_chooser.addOption("Turn Left", m_turnLeft);
      m_chooser.addOption("Shoot and Back", m_shootBack);
      m_chooser.addOption("Turn Shoot and Back", m_turnShootBack);
      m_chooser.addOption("Turn Shoot and BackMore", m_turnShootBackMore);

  
      // Put the chooser on the dashboard
      // Shuffleboard.getTab("Autonomous").add(m_chooser);
      Shuffleboard.getTab("Autonomous").add(m_chooser);
      
    }
  
    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
      // Grab the hatch when the 'A' button is pressed.
      // new JoystickButton(m_driverController, Button.kA.value)
          // .whenPressed(new GrabHatch(m_hatchSubsystem));
      
      // Autoaim
      new JoystickButton(m_mainStick, Button.kBumperRight.value)
        .whenPressed(
          new SequentialCommandGroup(
            new TurnToAngle(m_driveTrain).withTimeout(5)
          )
        );
      
      // Limelight LED Toggle
      new JoystickButton(m_mainStick, Button.kA.value)
        .whenPressed(
          new LimelightLightToggle()
        );

      // Limelight Camera Mode Toggle
      new JoystickButton(m_mainStick, Button.kB.value)
        .whenPressed(
          new LimelightCameraToggle()
        );

      // Limelight Stream View Toggle
      new JoystickButton(m_mainStick, Button.kY.value)
        .whenPressed(
          new LimelightStreamToggle()
        );

      // Take Pictures with Limelight
      new JoystickButton(m_mainStick, Button.kX.value)
        .whenPressed(
          new LimelightSnapshotToggle()
        );

      new JoystickButton(m_mainStick, Button.kStart.value)
        .whenPressed(
          new LimelightPipelineToggle()
        );
      
      // Run Intake In (Driver)
      new JoystickButton(m_mainStick, Button.kBumperLeft.value)
        .whileHeld(
          new IntakeControl(m_intake, -0.4 * 12)//-0.3,-0.5
        );

      // Run Intake In
      new JoystickButton(m_firstStick, 1)
        .whileHeld(
          new IntakeControl(m_intake, -0.4 * 12)//-0.3,-0.5
        );

      // Run Indexer Out
      new JoystickButton(m_firstStick, 2)
        .whileHeld(
          new IndexerControl(m_intake, 0.55 * 12)
        );

      // Run Indexer In
      new JoystickButton(m_firstStick, 3)
        .whileHeld(
          new IndexerControl(m_intake, -0.65 * 12)
        );

      // Run Intake Angle Down
      new JoystickButton(m_firstStick, 4)
        .whileHeld(
          new IntakeAngleControl(m_intake, 0.15 * 12)
        );

      // Run Intake Angle Up
      new JoystickButton(m_firstStick, 5)
        .whileHeld(
          new IntakeAngleControl(m_intake, -0.15 * 12)
        );

      // Run Everything Out
      new JoystickButton(m_firstStick, 7)
        .whileHeld(
          new ParallelCommandGroup(
            new IntakeControl(m_intake, 0.75 * 12),
            new IndexerControl(m_intake, 0.3 * 12),
            new FeederControl(m_feeder, -0.6 * 12)
          )
        );


      // Control Panel Position
      /*new JoystickButton(m_firstStick, 10)
        .whileHeld(
          new SpinnerControl(m_panel, 3, true)
        );
      
      // Control Panel Rotation
      new JoystickButton(m_firstStick, 11)
        .whileHeld(
          new SpinnerControl(m_panel, 0.25, false)
        );
      */
      
      //Control Panel Free Spin
      new JoystickButton(m_firstStick, 9)
        .whileHeld(
          new SpinnerControl(m_panel, 3, true)
        );

      //Control Panel Position
      new JoystickButton(m_firstStick, 10)
        .whenPressed(
          new PositionControl(m_panel)
        );

      //Control Panel Rotation
      new JoystickButton(m_firstStick, 11)
      .whenPressed(
        new RotationControl(m_panel)
      );
      
      // Belt Up to Shoot
      new JoystickButton(m_secondStick, 1)
        .whileHeld(
          new ParallelCommandGroup(
            new FeederControl(m_feeder, 0.8 * 12),
            new IndexerControl(m_intake, -0.3 * 12),
            new IntakeControl(m_intake, -0.75 * 12)
          )
        );

      // Run Intake Out
      new JoystickButton(m_secondStick, 2)
        .whileHeld(
          new IntakeControl(m_intake, 0.75 * 12)
        );

      // Run Shooter Mid Speed
      new JoystickButton(m_secondStick, 3)     // it was squared preserving sign so these are the true values from before
        .whileHeld(
          new ShooterControl(m_shooter, -0.5 * 12) // 0.5184 // -0.6
        );

      // Run Shooter Low Speed
      new JoystickButton(m_secondStick, 4)
        .whileHeld(
          new ShooterControl(m_shooter, -0.35 * 12) // 0.4096 // -0.38
        );

     
      // Run Shooter High Speed
      new JoystickButton(m_secondStick, 5)
        .whileHeld(
          new ShooterControl(m_shooter, -0.9 * 12)  // 0.81
        );
      
         /*
       // Run Shooter High Speed
       new JoystickButton(m_secondStick, 5)
       .whileHeld(
         new PIDShooterControl(m_shooter, -0.9)  // 0.81
       );
       */

      // Run Climber Down
      new JoystickButton(m_secondStick, 6)
        .whileHeld(
          new ClimbControl(m_climber, 0.5 * 12)
        );

      // Run Climber Up
      new JoystickButton(m_secondStick, 7)
        .whileHeld(
          new ClimbControl(m_climber, -0.5 * 12)
        );
    }
  
  
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
      /*
      var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(0,0,0), // kS, kV, kA
            AutoConstantsFinal.kDriveKinematics,
            10);

          // Create config for trajectory
      TrajectoryConfig config =
      new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                          AutoConstants.kMaxAccelerationMetersPerSecondSquared)
          // Add kinematics to ensure max speed is actually obeyed
          .setKinematics(AutoConstantsFinal.kDriveKinematics)
          // Apply the voltage constraint
          .addConstraint(autoVoltageConstraint);

      // An example trajectory to follow.  All units in meters.
      Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
          // Start at the origin facing the +X direction
          new Pose2d(0, 0, new Rotation2d(0)),
          // Pass through these two interior waypoints, making an 's' curve path
          List.of(
              new Translation2d(1, 1),
              new Translation2d(2, -1)
          ),
          // End 3 meters straight ahead of where we started, facing forward
          new Pose2d(3, 0, new Rotation2d(0)),
          // Pass config
          config
        );

      RamseteCommand ramseteCommand = new RamseteCommand(
          exampleTrajectory,
          m_driveTrain::getPose,
          new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
          new SimpleMotorFeedforward(0,0,0),
          AutoConstantsFinal.kDriveKinematics,
          m_driveTrain::getWheelSpeeds,
          new PIDController(0, 0, 0), // DriveConstants.kPDriveVel, 0, 0
          new PIDController(0, 0, 0),
          // RamseteCommand passes volts to the callback
          m_driveTrain::tankDriveVolts,
          m_driveTrain
        );

      // Run path following command, then stop at the end.
      return ramseteCommand.andThen(() -> m_driveTrain.tankDriveVolts(0, 0));
      */

      return m_chooser.getSelected().withTimeout(15);
    }
}