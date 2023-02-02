package frc.team7520.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team7520.robot.subsystems.SwerveBase;


public class TeleopDrive extends CommandBase {

    XboxController controller = new XboxController(0);

    public SwerveBase swerveBase;

    public TeleopDrive() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)

        swerveBase = SwerveBase.getInstance();

        addRequirements( swerveBase );
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        swerveBase.drive(controller.getLeftX(), controller.getLeftY(), controller.getRightX());
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
