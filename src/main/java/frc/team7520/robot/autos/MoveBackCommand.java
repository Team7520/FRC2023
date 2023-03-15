package frc.team7520.robot.autos;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team7520.robot.RobotContainer;
import frc.team7520.robot.subsystems.SwerveBase;


public class MoveBackCommand extends CommandBase {

    public MoveBackCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements();
    }

    @Override
    public void initialize() {

        RobotContainer.swerve1.resetEncoders();
        RobotContainer.swerve2.resetEncoders();
        RobotContainer.swerve3.resetEncoders();
        RobotContainer.swerve4.resetEncoders();

        SwerveBase swerveBase = SwerveBase.getInstance();

        SwerveModuleState swerveModuleState = new SwerveModuleState();

        swerveBase.setModuleStates(new SwerveModuleState[]{swerveModuleState, swerveModuleState, swerveModuleState, swerveModuleState});

    }

    @Override
    public void execute() {

        RobotContainer.swerve1.setSwerve(0,0,false);
        RobotContainer.swerve2.setSwerve(0,0,false);
        RobotContainer.swerve3.setSwerve(0,0,false);
        RobotContainer.swerve4.setSwerve(0,0,false);

        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        double distance = .2;

        RobotContainer.swerve1.driveMotor.set(distance);/*.setReference(distance, CANSparkMax.ControlType.kPosition);*/
        RobotContainer.swerve2.driveMotor.set(distance);/*.setReference(distance, CANSparkMax.ControlType.kPosition);*/
        RobotContainer.swerve3.driveMotor.set(distance);/*.setReference(distance, CANSparkMax.ControlType.kPosition);*/
        RobotContainer.swerve4.driveMotor.set(distance);/*.setReference(distance, CANSparkMax.ControlType.kPosition);*/

        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        distance = .5;

        RobotContainer.swerve1.driveMotor.set(distance);/*.setReference(distance, CANSparkMax.ControlType.kPosition);*/
        RobotContainer.swerve2.driveMotor.set(distance);/*.setReference(distance, CANSparkMax.ControlType.kPosition);*/
        RobotContainer.swerve3.driveMotor.set(distance);/*.setReference(distance, CANSparkMax.ControlType.kPosition);*/
        RobotContainer.swerve4.driveMotor.set(distance);/*.setReference(distance, CANSparkMax.ControlType.kPosition);*/

        try {
            Thread.sleep(1950);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }


        RobotContainer.swerve1.setSwerve(315,0,true);
        RobotContainer.swerve2.setSwerve(225,0,true);
        RobotContainer.swerve3.setSwerve(135,0,true);
        RobotContainer.swerve4.setSwerve(45,0,true);

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
