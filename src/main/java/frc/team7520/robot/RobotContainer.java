// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team7520.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team7520.robot.Constants.OperatorConstants;
import frc.team7520.robot.commands.TeleopDrive;
import frc.team7520.robot.subsystems.Arm;
import frc.team7520.robot.subsystems.NavXGyro;
import frc.team7520.robot.subsystems.SwerveModule;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;




/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    public static final SwerveModule swerve1 = new SwerveModule(1, 11, Constants.SwerveConstants.invertDrive, false);
    public static final SwerveModule swerve2 = new SwerveModule(2, 12, Constants.SwerveConstants.invertDrive, true);
    public static final SwerveModule swerve3 = new SwerveModule(3, 13, Constants.SwerveConstants.invertDrive, false);
    public static final SwerveModule swerve4 = new SwerveModule(4, 14, Constants.SwerveConstants.invertDrive, false);
    public static final CANSparkMax armMotor = new CANSparkMax(40,CANSparkMaxLowLevel.MotorType.kBrushless);
    public static final CANSparkMax elbowMotor = new CANSparkMax(41,CANSparkMaxLowLevel.MotorType.kBrushless);
    public static final DigitalInput input = new DigitalInput(0);
    public static final DigitalInput photoSwitch = new DigitalInput(1);
    // The robot's subsystems and commands are defined here...

    public static final NavXGyro _navXGyro = new NavXGyro();
    public final TeleopDrive teleopDrive = new TeleopDrive(new XboxController(OperatorConstants.DRIVER_CONTROLLER_PORT), _navXGyro);


    public final Arm arm = Arm.getInstance();


    // Replace with CommandPS4Controller or CommandJoystick if needed
    public static final XboxController driverController =
            new XboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
    public final CommandXboxController operatorController =
            new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        // Configure the trigger bindings
        configureBindings();
    }


    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings()
    {


    }
}
