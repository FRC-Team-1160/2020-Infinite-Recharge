package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.drive.TurnToAngle;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.DriveTrain;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    public final DriveTrain m_driveTrain; 

    public final Delivery m_delivery;

    // The autonomous routines
  
    // A simple auto routine that drives forward a specified distance, and then stops.
    //private final Command m_simpleAuto =
        //new DriveDistance(AutoConstants.kAutoDriveDistanceInches, AutoConstants.kAutoDriveSpeed,
                         // m_robotDrive);
  
    // A chooser for autonomous commands
    SendableChooser<Command> m_chooser = new SendableChooser<>();
  
    // The driver's controller
    private Joystick m_mainStick = new Joystick(OIConstants.mainStickPort);
  
    // Secondary controllers
    private Joystick m_firstStick = new Joystick(OIConstants.firstStickPort);

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
      m_driveTrain = DriveTrain.getInstance();

      m_delivery = Delivery.getInstance();

      // Configure the button bindings
      configureButtonBindings();
  
      // Configure default commands
      
      m_driveTrain.setDefaultCommand(new RunCommand(
        () -> m_driveTrain.tankDrive((m_mainStick.getRawAxis(1)) , (m_mainStick.getRawAxis(4)), 0.0),
        m_driveTrain)
      );

      m_delivery.setDefaultCommand(new RunCommand(
        () -> m_delivery.shoot(m_firstStick.getRawAxis(2), m_firstStick.getRawAxis(1), m_mainStick.getRawAxis(2), m_mainStick.getRawAxis(3)),
        m_delivery)
      );
  
      // Add commands to the autonomous command chooser
      // m_chooser.addOption("Simple Auto", m_simpleAuto);
  
      // Put the chooser on the dashboard
      // Shuffleboard.getTab("Autonomous").add(m_chooser);
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
      new JoystickButton(m_mainStick, 1)
        .whenPressed(
          new SequentialCommandGroup(
            //             PID Controller               Double Supplier     Setpoint      Double Consumer     Requirement
            //new PIDCommand(m_driveTrain.turnController, m_driveTrain,       90.0f,        m_driveTrain,       m_driveTrain);
            new TurnToAngle(m_driveTrain)
          )
        );
    }
  
  
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
      return m_chooser.getSelected();
    }
}