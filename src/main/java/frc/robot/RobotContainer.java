package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.climb.ClimbControl;
import frc.robot.commands.drive.Drive;
import frc.robot.commands.drive.TurnToAngle;
import frc.robot.commands.feed.FeederControl;
import frc.robot.commands.intake.IndexerControl;
import frc.robot.commands.intake.IntakeAngleControl;
import frc.robot.commands.intake.IntakeControl;
import frc.robot.commands.panel.PositionControl;
import frc.robot.commands.panel.RotationControl;
import frc.robot.commands.panel.SpinnerControl;
import frc.robot.commands.shoot.ShootGroupControl;
import frc.robot.commands.shoot.ShooterControl;
import frc.robot.commands.vision.LimelightCameraToggle;
import frc.robot.commands.vision.LimelightLightToggle;
import frc.robot.commands.vision.LimelightSnapshotToggle;
import frc.robot.commands.vision.LimelightStreamToggle;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Panel;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

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
      new Drive(m_driveTrain, 0.5).withTimeout(2)
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
        new TurnToAngle(m_driveTrain),
        new ShootGroupControl(m_shooter, -0.61 * 12, m_feeder, 0.3 * 12).withTimeout(5.0),
        new Drive(m_driveTrain, 0.5).withTimeout(1) 
      );

      private final Command m_shootForward =
        new SequentialCommandGroup(
        //  new ParallelCommandGroup(
        //    new ShooterControl(m_shooter, -0.9),
        //    new RunCommand(
        //       () -> m_feeder.feederControl(0.3),
        //       m_feeder)
        //  ).withTimeout(5.0),
        new ShootGroupControl(m_shooter, -0.41 * 12, m_feeder, 0.3 * 12).withTimeout(5.0),
        new Drive(m_driveTrain, -0.5).withTimeout(2)
      );

      private final Command m_forwardShoot =
        new SequentialCommandGroup(
        //  new ParallelCommandGroup(
        //    new ShooterControl(m_shooter, -0.9),
        //    new RunCommand(
        //       () -> m_feeder.feederControl(0.3),
        //       m_feeder)
        //  ).withTimeout(5.0),
        new Drive(m_driveTrain, -0.5).withTimeout(2),
        new ShootGroupControl(m_shooter, -0.41 * 12, m_feeder, 0.3 * 12).withTimeout(5.0)
      );

      private final Command m_shootTrenchShoot = 
        new SequentialCommandGroup(
          new TurnToAngle(m_driveTrain),
          new ShootGroupControl(m_shooter, -0.41 * 12, m_feeder, 0.3 * 12).withTimeout(4.0),
          new TurnToAngle(m_driveTrain, 0),
          new IntakeAngleControl(m_intake, 0.15 * 12).withTimeout(0.5),
          new ParallelCommandGroup(
            new Drive(m_driveTrain, 0.5),
            new FeederControl(m_feeder, 0.8 * 12),
            new IndexerControl(m_intake, -0.3 * 12),
            new IntakeControl(m_intake, -0.75 * 12)
          ).withTimeout(2.0),
          new Drive(m_driveTrain, -0.75).withTimeout(1.0),
          new TurnToAngle(m_driveTrain),
          new ShootGroupControl(m_shooter, -0.41 * 12, m_feeder, 0.3 * 12).withTimeout(4.0)
        );

    private final Command m_back = new Drive(m_driveTrain, 0.5).withTimeout(2); // no * 12 because it is still .set, not .setVoltage

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

      m_chooser.addOption("Shoot and Back", m_shootBack);
      m_chooser.addOption("Auto-Align Shoot and Back", m_turnShootBack);
      m_chooser.addOption("Shoot and Forward", m_shootForward);
      m_chooser.addOption("Forward and Shoot", m_forwardShoot);
      m_chooser.addOption("Back", m_back);
      m_chooser.addOption("Shoot, Trench, and Shoot", m_shootTrenchShoot);
  
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
      
      
      new JoystickButton(m_mainStick, Button.kBumperRight.value)
        .whenPressed(
          new SequentialCommandGroup(
            new TurnToAngle(m_driveTrain)
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

      
      // Run Intake In (Driver)
      new JoystickButton(m_mainStick, Button.kBumperLeft.value)
        .whileHeld(
          new IntakeControl(m_intake, -0.4 * 12)//-0.3,-0.5
        );

      // Run Intake In
      new JoystickButton(m_firstStick, 1)
        .whileHeld(
          new IntakeControl(m_intake, -0.6 * 12)//-0.3,-0.5
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

      // Run Intake Out
      new JoystickButton(m_secondStick, 2)
        .whileHeld(
          new IntakeControl(m_intake, 0.75 * 12)
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

      /*
      //Control Panel Position
      new JoystickButton(m_firstStick, 10)
        .whenPressed(
          new PositionControl(m_panel, 1.0, true)
        );

      //Control Panel Rotation
      new JoystickButton(m_firstStick, 11)
      .whenPressed(
        new RotationControl(m_panel, 1.0, true)
      );
      */
      
      // Belt Up to Shoot
      new JoystickButton(m_secondStick, 8)
        .whileHeld(
          new ParallelCommandGroup(
            new FeederControl(m_feeder, 0.8 * 12),
            new IndexerControl(m_intake, -0.3 * 12),
            new IntakeControl(m_intake, -0.75 * 12)
          )
        );

      // new JoystickButton(m_secondStick, 1)
      //   .whileHeld(
      //     new IntakeControl(m_intake, -0.7 * 12)//-0.3,-0.5
      //   );

      // Run Shooter Mid Speed
      new JoystickButton(m_secondStick, 3)     // it was squared preserving sign so these are the true values from before
        .whileHeld(
          new ShooterControl(m_shooter, -0.41 * 12) // 0.5184 // -0.6 // -0.5
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
      return m_chooser.getSelected().withTimeout(15);
    }
}