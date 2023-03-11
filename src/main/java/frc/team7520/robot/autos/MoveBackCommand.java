package frc.team7520.robot.autos;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team7520.robot.RobotContainer;


public class MoveBackCommand extends CommandBase {

    public MoveBackCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements();
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        System.out.println("e");

        RobotContainer.swerve1.driveMotor.getPIDController().setReference(26.66, CANSparkMax.ControlType.kPosition);
        RobotContainer.swerve2.driveMotor.getPIDController().setReference(26.66, CANSparkMax.ControlType.kPosition);
        RobotContainer.swerve3.driveMotor.getPIDController().setReference(26.66, CANSparkMax.ControlType.kPosition);
        RobotContainer.swerve4.driveMotor.getPIDController().setReference(26.66, CANSparkMax.ControlType.kPosition);

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
