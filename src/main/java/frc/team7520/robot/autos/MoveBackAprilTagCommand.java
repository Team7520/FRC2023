package frc.team7520.robot.autos;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team7520.robot.RobotContainer;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;


public class MoveBackAprilTagCommand extends CommandBase {

    private final double distance;

    public MoveBackAprilTagCommand(double distance) {
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

        PhotonCamera camera = new PhotonCamera("photonvision");

        var result = camera.getLatestResult();

        if(false) {
            List<PhotonTrackedTarget> targets = result.getTargets();

            PhotonTrackedTarget target = result.getBestTarget();

            Transform3d bestCameraToTarget = target.getBestCameraToTarget();
        } else {
            double pos = 83.54/*(distance) / 0.0119694680101771122385426712902949059887912154116191531779145388*/;

            RobotContainer.swerve1.driveMotor.getPIDController().setReference(pos, CANSparkMax.ControlType.kPosition);
            RobotContainer.swerve2.driveMotor.getPIDController().setReference(pos, CANSparkMax.ControlType.kPosition);
            RobotContainer.swerve3.driveMotor.getPIDController().setReference(pos, CANSparkMax.ControlType.kPosition);
            RobotContainer.swerve4.driveMotor.getPIDController().setReference(pos, CANSparkMax.ControlType.kPosition);

            return;
        }
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
