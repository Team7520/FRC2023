//package frc.team7520.robot.autos;
//
//import com.pathplanner.lib.auto.SwerveAutoBuilder;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj2.command.PrintCommand;
//import frc.team7520.robot.RobotContainer;
//
//import java.util.HashMap;
//
//
//public class ChargeCommand extends CommandBase {
//
//    public ChargeCommand() {
//        // each subsystem used by the command must be passed into the
//        // addRequirements() method (which takes a vararg of Subsystem)
//        addRequirements();
//    }
//
//    @Override
//    public void initialize() {
//
//        // This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
//// for every path in the group
//        ArrayList<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("FullAuto", new PathConstraints(4, 3));
//
//// This is just an example event map. It would be better to have a constant, global event map
//// in your code that will be used by all path following commands.
//        HashMap<String, Command> eventMap = new HashMap<>();
//        eventMap.put("marker1", new PrintCommand("Passed marker 1"));
//
//// Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
//        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
//                RobotContainer.swerveBase::getPose, // Pose2d supplier
//                RobotContainer.swerveBase::resetPose, // Pose2d consumer, used to reset odometry at the beginning of auto
//                RobotContainer.swerveBase.kinematics, // SwerveDriveKinematics
//                new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
//                new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
//                RobotContainer.swerveBase::setModuleStates, // Module states consumer used to output to the drive subsystem
//                eventMap,
//                true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
//                RobotContainer.swerveBase // The drive subsystem. Used to properly set the requirements of path following commands
//        );
//
//    }
//
//    @Override
//    public void execute() {
//
//    }
//
//    @Override
//    public boolean isFinished() {
//        // TODO: Make this return true when this Command no longer needs to run execute()
//        return false;
//    }
//
//    @Override
//    public void end(boolean interrupted) {
//
//    }
//}
