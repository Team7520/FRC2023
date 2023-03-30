package frc.team7520.robot.autos;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team7520.robot.RobotContainer;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;


public class MoveBackCommand extends CommandBase {

    private final double distance;

    public MoveBackCommand(double distance) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)

        this.distance = distance;

        addRequirements();
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double speed = 0.5;
        RobotContainer.swerve1.driveMotor.set(speed);/*.setReference(speed, CANSparkMax.ControlType.kPosition);*/
        RobotContainer.swerve2.driveMotor.set(speed);/*.setReference(speed, CANSparkMax.ControlType.kPosition);*/
        RobotContainer.swerve3.driveMotor.set(speed);/*.setReference(speed, CANSparkMax.ControlType.kPosition);*/
        RobotContainer.swerve4.driveMotor.set(speed);/*.setReference(speed, CANSparkMax.ControlType.kPosition);*/

        try {
            Thread.sleep(5000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        RobotContainer.swerve1.stop();
        RobotContainer.swerve2.stop();
        RobotContainer.swerve3.stop();
        RobotContainer.swerve4.stop();

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
