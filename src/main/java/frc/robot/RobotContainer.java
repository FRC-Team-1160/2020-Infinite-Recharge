package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.*;
import frc.robot.commands.panel.*;
import frc.robot.commands.drive.*;
import frc.robot.subsystems.Panel;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final Panel m_panel = Panel.getInstance();
  
    // A chooser for autonomous commands
    SendableChooser<Command> m_chooser = new SendableChooser<>();
  
    // The driver's controller
    Joystick m_mainStick = new Joystick(OIConstants.mainStickPort);
  
    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
      // Configure the button bindings
      configureButtonBindings();
  
      // Configure default commands
    }
  
    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
      // Rotate the control panel 3-5 times
      new JoystickButton(m_mainStick, 0)
        .whenPressed(
          new RotationControl(m_panel)
        );

      // Spin the control panel to the target color
      new JoystickButton(m_mainStick, 1)
      .whenPressed(
        new PositionControl(m_panel)
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