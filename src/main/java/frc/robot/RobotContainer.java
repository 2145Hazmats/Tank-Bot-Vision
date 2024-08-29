package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
    // Instantiate Subsystems
    private Drivetrain m_Drivetrain = new Drivetrain();
    // Operator controllers
    private final CommandXboxController m_driverController =
            new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER);
    private final CommandXboxController m_operatorController =
            new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER);

    public RobotContainer() {
        // Allows choosing the drive type through SmartDashboard
        SendableChooser<String> m_driveSendableChooser = new SendableChooser<String>();
        m_driveSendableChooser.setDefaultOption("arcadeDrive",  "arcadeDrive");
        m_driveSendableChooser.addOption("tankDrive", "tankDrive");
        m_driveSendableChooser.addOption("curvatureDrive1", "curvatureDrive1");
        m_driveSendableChooser.addOption("curvatureDrive2", "curvatureDrive2");
        SmartDashboard.putData("Drive Type", m_driveSendableChooser);

        // Default Drivetrain command. Calls the "drive" method in Drivetrain
        // Controller inputs and the selected drive type are passed as arguments
        m_Drivetrain.setDefaultCommand(m_Drivetrain.drive(
                m_driverController::getLeftY,
                m_driverController::getRightX,
                m_driverController::getRightY,
                m_driveSendableChooser::getSelected
        ));

        configureBindings();
    }

    private void configureBindings() {
        // Implementation of various vision methods. The parameter "desiredRange" can be freely changed
        m_driverController.a().whileTrue(m_Drivetrain.rotateToTarget(m_driverController::getLeftY, m_driverController::getRightX));
        m_driverController.b().whileTrue(m_Drivetrain.driveToTarget(m_driverController::getLeftY, m_driverController::getRightX, 1.5));
        m_driverController.x().whileTrue(m_Drivetrain.driveRotateToTarget(m_driverController::getLeftY, m_driverController::getRightX, 1.5));
    }

    public Command getAutonomousCommand() {
        return null;
    }

}
