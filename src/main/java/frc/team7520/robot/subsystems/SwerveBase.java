package frc.team7520.robot.subsystems;


import com.ctre.phoenix.motorcontrol.WPI_AutoFeedEnable;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveBase extends SubsystemBase {

    public final double L = 1;
    public final double W = 1;

    // With eager singleton initialization, any static variables/fields used in the
    // constructor must appear before the "INSTANCE" variable so that they are initialized
    // before the constructor is called when the "INSTANCE" variable initializes.

    /**
     * The Singleton instance of this SwerveBase. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static SwerveBase INSTANCE = new SwerveBase();

    /**
     * Returns the Singleton instance of this SwerveBase. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code SwerveBase.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static SwerveBase getInstance() {
        return INSTANCE;
    }

    SwerveModule swerveModule1;
    SwerveModule swerveModule2;
    SwerveModule swerveModule3;
    SwerveModule swerveModule4;

    /**
     * Creates a new instance of this SwerveBase. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private SwerveBase() {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.

        swerveModule1 = new SwerveModule(1, 11, 0,1);
        swerveModule2 = new SwerveModule(2, 12, 2,3);
        swerveModule3 = new SwerveModule(3, 13, 4,5);
        swerveModule4 = new SwerveModule(4, 14, 6,7);
        addChild("swerveModule1", swerveModule1);
        addChild("swerveModule2", swerveModule2);
        addChild("swerveModule3", swerveModule3);
        addChild("swerveModule4", swerveModule4);
    }

    public void drive(double x1, double y1, double x2) {
        double r = Math.sqrt((L * L) + (W * W));

        x1 = -x1;

        double a = (x1 - x2 * (L / r));
        double b = (x1 + x2 * (L / r));
        double c = y1 - x2 * (W / r);
        double d = y1 + x2 * (W / r);

        double speed1 = Math.sqrt((a * a) + (d * d));
        double speed2 = Math.sqrt((a * a) + (c * c));
        double speed3 = Math.sqrt((b * b) + (d * d));
        double speed4 = Math.sqrt((b * b) + (c * c));

        Rotation2d angle1 = Rotation2d.fromRadians(Math.atan2(a, d));
        Rotation2d angle2 = Rotation2d.fromRadians(Math.atan2(a, c));
        Rotation2d angle3 = Rotation2d.fromRadians(Math.atan2(b, d));
        Rotation2d angle4 = Rotation2d.fromRadians(Math.atan2(b, c));

        //TODO: Remove.  This sets all wheels to the same speed
        swerveModule1.drive(speed1);
        swerveModule2.drive(speed2);
        swerveModule3.drive(speed3);
        swerveModule4.drive(speed4);

        SmartDashboard.putNumber("speed1", speed1);
        SmartDashboard.putNumber("speed2", speed2);
        SmartDashboard.putNumber("speed3", speed3);
        SmartDashboard.putNumber("speed4", speed4);

        swerveModule1.turnToAngle(angle1);
        swerveModule2.turnToAngle(angle2);
        swerveModule3.turnToAngle(angle3);
        swerveModule4.turnToAngle(angle4);
    }

    public CommandBase resetEncoders() {

        return runOnce(() -> {
            swerveModule1.resetEncoder();
            swerveModule2.resetEncoder();
            swerveModule3.resetEncoder();
            swerveModule4.resetEncoder();

        });
    }
}

