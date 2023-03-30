// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team7520.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team7520.robot.Constants.OperatorConstants;
import frc.team7520.robot.commands.TeleopDrive;
import frc.team7520.robot.subsystems.*;
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

    public static final NavXGyro _navXGyro = NavXGyro.getInstance();
    public final TeleopDrive teleopDrive = new TeleopDrive(new XboxController(OperatorConstants.DRIVER_CONTROLLER_PORT), _navXGyro);

    // Replace with CommandPS4Controller or CommandJoystick if needed
    public static final XboxController driverController =
            new XboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
    public final XboxController operatorController =
            new XboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

    JoystickButton operatorLeftBumper = new JoystickButton(operatorController, Button.kLeftBumper.value);
    JoystickButton operatorRightModifier = new JoystickButton(operatorController, Button.kStart.value);
    JoystickButton operatorLeftModifier = new JoystickButton(operatorController, Button.kBack.value);
    JoystickButton operatorRightBumper = new JoystickButton(operatorController, Button.kRightBumper.value);
    JoystickButton operatorYButton = new JoystickButton(operatorController, Button.kY.value);
    JoystickButton operatorAButton = new JoystickButton(operatorController, Button.kA.value);
    JoystickButton operatorBButton = new JoystickButton(operatorController, Button.kB.value);
    JoystickButton operatorXButton = new JoystickButton(operatorController, Button.kX.value);

    JoystickButton driverYButton = new JoystickButton(driverController, Button.kY.value);
    JoystickButton driverAButton = new JoystickButton(driverController, Button.kA.value);
    JoystickButton driverBButton = new JoystickButton(driverController, Button.kB.value);
    JoystickButton driverXButton = new JoystickButton(driverController, Button.kX.value);

    public static Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
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

        System.out.println("Bind");

        Arm.getInstance().setDefaultCommand(Arm.getInstance().moveArm());

        operatorYButton.whileTrue(Arm.getInstance().rest());
        operatorAButton.whileTrue(Arm.getInstance().cone());
        operatorBButton.whileTrue(Arm.getInstance().cube());
        operatorXButton.whileTrue(Arm.getInstance().floor());

        operatorLeftBumper.whileTrue(Hand.getInstance().openHand().repeatedly());

        driverBButton.whileTrue(SwerveBase.getInstance().lock());

//        operatorRightBumper.whileTrue(Arm.getInstance().toggle());

    }
}
