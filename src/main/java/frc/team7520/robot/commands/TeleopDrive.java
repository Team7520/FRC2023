// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team7520.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team7520.robot.Constants;
import frc.team7520.robot.subsystems.SwerveBase;
import frc.team7520.robot.subsystems.NavXGyro;

public class TeleopDrive extends CommandBase {

    private SwerveBase _swerveBase;
    private XboxController swerveBaseController;
    private Joystick leftStick;
    private Joystick rightStick;
    private NavXGyro _navXGyro;
    double stickForward;
    double stickStrafe;
    double stickRotate;

    public static final double OMEGA_SCALE = 1/1.5d;
    public static final double DEADZONE_LSTICK = 0.07;
    private static final double DEADZONE_RSTICK = 0.07;
    private double originHeading = 0.0;
    private double originCorr;
    private double leftPow = 1;
    private double rightPow = 1;

    /** Creates a new SwerveBase. */
    public TeleopDrive(XboxController swerveBaseController, NavXGyro gyro) {
        this._swerveBase = SwerveBase.getInstance(gyro);
        this.swerveBaseController = swerveBaseController;
        this._navXGyro = gyro;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(_swerveBase);
    }

    public TeleopDrive(Joystick leftStick, Joystick rightStick, NavXGyro gyro) {
        this._swerveBase = SwerveBase.getInstance(gyro);
        this.leftStick = leftStick;
        this.rightStick = rightStick;
        this._navXGyro = gyro;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(_swerveBase);
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        originHeading = _navXGyro.getZeroAngle();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        stickForward = Math.abs(this.swerveBaseController.getLeftY()) > Constants.deadZone ? this.swerveBaseController.getLeftY() : 0d;
        stickStrafe = Math.abs(this.swerveBaseController.getLeftX()) > Constants.deadZone ? this.swerveBaseController.getLeftX() : 0d;
        stickRotate = Math.abs(this.swerveBaseController.getRightX()) > Constants.deadZone ? this.swerveBaseController.getRightX() : 0d;

        double strafe = Math.pow(Math.abs(stickStrafe), leftPow) * Math.signum(stickStrafe);
        double forward = Math.pow(Math.abs(stickForward), leftPow) * Math.signum(stickForward);
        double omega = Math.pow(Math.abs(stickRotate), rightPow) * Math.signum(stickRotate) * OMEGA_SCALE;

        if (Math.abs(strafe) < DEADZONE_LSTICK)
            strafe = 0.0;
        if (Math.abs(forward) <DEADZONE_LSTICK)
            forward = 0.0;
        if (Math.abs(omega) < DEADZONE_RSTICK * OMEGA_SCALE)
            omega = 0.0;
        boolean stickFieldCentric = swerveBaseController.getLeftBumper();
        boolean swerveBaseCorrect = true;

        if (!stickFieldCentric) {
            // The calculations correct the forward and strafe values for field centric
            // attitude.

            // Rotate the velocity vector from the joystick by the difference between our
            // current orientation and the current origin heading
            // final double originCorrection = Math.toRadians(originHeading - Navx.getInstance().navX.getFusedHeading());
            // final double temp = forward * Math.cos(originCorrection) - strafe * Math.sin(originCorrection);
            final double originCorrection = Math.toRadians(originHeading - _navXGyro.getNavAngle());
            final double temp = forward * Math.cos(originCorrection) + strafe * Math.sin(originCorrection);
            strafe = strafe * Math.cos(originCorrection) - forward * Math.sin(originCorrection);
            forward = temp;
        }

        // If all of the joysticks are in the deadzone, don't update the motors
        // This makes side-to-side strafing much smoother
        boolean deadStick = false;
        if (Math.abs(forward) < Constants.deadZone && Math.abs(strafe) < Constants.deadZone && Math.abs(omega) < Constants.deadZone) {
            deadStick = true;
        }

        SmartDashboard.putBoolean("Dead Stick", deadStick);

        // SmartDashboard.putNumber("Forward Done", forward);
        // SmartDashboard.putNumber("Strafe Done", strafe);
        // SmartDashboard.putNumber("Rotation Done", omega);


        this._swerveBase.processInput(forward, strafe, omega, deadStick, swerveBaseCorrect);

        // this._swerveBase.processInput(
        // () -> 0.0,
        // () -> 0.0,
        // () -> 0.0
        // );
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
