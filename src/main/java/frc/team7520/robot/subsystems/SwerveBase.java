// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team7520.robot.subsystems;

import java.util.Arrays;
import java.util.Collections;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import frc.team7520.robot.subsystems.NavXGyro;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team7520.robot.Constants;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class SwerveBase extends SubsystemBase {

    private static SwerveBase instance;

    public static final double kMaxSpeed = 3.0; // 3 meters per second
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

    private static SwerveModule swerve1;
    private static SwerveModule swerve2;
    private static SwerveModule swerve3;
    private static SwerveModule swerve4;

    //public static AHRS navX;
    public double heading;
    public double angle;

    private static final double WHEEL_BASE_LENGTH = 1;//23; // 30;
    private static final double WHEEL_BASE_WIDTH = 1;//21.5; // 28;

    //private static final double WHEEL_DIAMETER = 4.0;
    // TO DO: Correct equation that uses MAX_SPEED
    public static final double MAX_SPEED = 0.75; // Max speed is 0 to 1
    public static final double MAX_REVERSIBLE_SPEED_DIFFERENCE = 0.7 * MAX_SPEED;

    public static final double OMEGA_SCALE = 1.0 / 30.0;

    private final boolean invertDrive = true;//false;
    private final boolean invertSteer = true;
    private NavXGyro _gyro;
    private boolean _driveCorrect;

    private ShuffleboardTab driveTab = Shuffleboard.getTab("DriveTab");

    private SwerveDriveOdometry odometer;


    //SwerveModule(int steerNum, int driveNum, boolean invertDrive, boolean invertSteer)

    public SwerveBase(NavXGyro gyro) {

        this._gyro = gyro;

        swerve1 = new SwerveModule(1, 11, invertDrive, false);

        swerve2 = new SwerveModule(2, 12, invertDrive, true);

        swerve3 = new SwerveModule(3, 13, invertDrive, false);

        swerve4 = new SwerveModule(4, 14, invertDrive, false);

        odometer = new SwerveDriveOdometry(Constants.kDriveKinematics,
                new Rotation2d(0),
                getModulePositions());

    }

    // Public Methods
    // public static Drive getInstance(double width, double length) {
    //   if (instance == null) {
    //     instance = new Drive(width, length);
    //   }
    //   return instance;
    // }
    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(this._gyro.getRotation2d(), getModulePositions(), pose);
    }

    public static SwerveBase getInstance(NavXGyro gyro) {
        if (instance == null) {
            instance = new SwerveBase(gyro);
        }

        return instance;
    }

    public void stopFrontLeft() {
        swerve1.stopDriveMotor();
    }

    public void stopBackLeft() {
        swerve2.stopDriveMotor();
    }

    public void stopFrontRight() {
        swerve3.stopDriveMotor();
    }

    public void stopBackRight() {
        swerve4.stopDriveMotor();
    }

    public void setFrontLeft(double speed) {
        swerve1.setDriveSpeed(speed);
    }

    public void setBackLeft(double speed) {
        swerve2.setDriveSpeed(speed);
    }

    public void setFrontRight(double speed) {
        swerve3.setDriveSpeed(speed);
    }

    public void setBackRight(double speed) {
        swerve4.setDriveSpeed(speed);
    }

    GenericEntry lfSetAngle = driveTab.addPersistent("LF Set Angle", 0)
            .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -180, "max", 180, "center", 0))
            .withPosition(0, 0).withSize(3, 1).getEntry();

    GenericEntry lbSetAngle = driveTab.addPersistent("LB Set Angle", 0)
            .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -180, "max", 180, "center", 0))
            .withPosition(0, 1).withSize(3, 1).getEntry();

    GenericEntry rfSetAngle = driveTab.addPersistent("RF Set Angle", 0)
            .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -180, "max", 180, "center", 0))
            .withPosition(4, 0).withSize(3, 1).getEntry();

    GenericEntry rbSetAngle = driveTab.addPersistent("RBack Set Angle", 0)
            .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -180, "max", 180, "center", 0))
            .withPosition(4, 2).withSize(3, 1).getEntry();

    public void processInput(double forward, double strafe, double omega, boolean deadStick, boolean driveCorrect) {

        this._driveCorrect = driveCorrect;
        double omegaL2 = omega * (WHEEL_BASE_LENGTH / 2.0);
        //SmartDashboard.putNumber("OmegaL2", omegaL2);
        double omegaW2 = omega * (WHEEL_BASE_WIDTH / 2.0);
        SmartDashboard.putNumber("OmegaW2", omegaW2);

        SmartDashboard.putNumber("Forwrad", forward);
        SmartDashboard.putNumber("Strafe", strafe);

        // Compute the constants used later for calculating speeds and angles
        double A = strafe - omega;
        double B = strafe + omega;
        double C = forward - omega;
        double D = forward + omega;

                /*
         * ... and angles for the steering motors Set the drive to face straight ahead
         * and then either mechanically set the encoders to read zero, or mathematically
         * correct the angle by reading the encoder value when the drive is pointed
         * straight ahead and adding or subtracting that value from the reading
         */

        /*
         * Get offset values from the driver station using NetworkTables. Values are
         * then input to "calibrate" the position of the drives mathematically rather
         * then by mechanically positioning the drives and physically setting the
         * encoder to zero.
         */

        double lfOffset = lfSetAngle.getDouble(0.0);
        double lbOffset = lbSetAngle.getDouble(0.0);
        double rfOffset = rfSetAngle.getDouble(0.0);
        double rbOffset = rbSetAngle.getDouble(0.0);


        // Compute the drive motor speeds
        // double speedFL = speed(B, D);
        // double speedBL = speed(A, D);
        // double speedFR = speed(B, C);
        // double speedBR = speed(A, C);
        double speedFL = speed(B, C);
        double speedBL = speed(A, C);
        double speedFR = speed(A, D);
        double speedBR = speed(B, D);

        /*
         * When drives are mechanically calibrated for zero position on encoders they
         * can be at 90 degrees to the front of the robot. Adding or subtracting 90
         * degrees to the steering calculation can be used offset for initial
         * position/calibration of the drives.
         *
         * For swerve and steer drives constants are 90 degrees out of phase when they
         * are inserted in frames sideways. angleFL - 90 angleBL + 90 angleFR - 90
         * angleBR + 90
         */


        // Angles for the steering motors
        // When drives are calibrated for zero position on encoders
        // They are at 90 degrees to the front of the robot.
        // Subtract and add 90 degrees to steering calculation to offset for initial
        // position/calibration of drives.
        double angleFL = angle(B, C);
        double angleBL = angle(A, C);
        double angleFR = angle(A, D);
        double angleBR = angle(B, D);

        // Compute the maximum speed so that we can scale all the speeds to the range
        // [0.0, 1.0]
        double maxSpeed = Collections.max(Arrays.asList(speedFL, speedBL, speedFR, speedBR, 1.0));

        // Set each swerve module, scaling the drive speeds by the maximum speed

        SmartDashboard.putNumber("angleLF", angleFL);
        SmartDashboard.putNumber("speedLF", speedFL);
        SmartDashboard.putNumber("CurAngle FL", swerve1.getSteerEncDeg());
        // SmartDashboard.putNumber("angleRF", angleFR);
        // SmartDashboard.putNumber("speedRF", speedFR);
        // SmartDashboard.putNumber("CurAngle FR", frontRight.getSteerEncDeg());
        // SmartDashboard.putNumber("angleLR", angleBL);
        // SmartDashboard.putNumber("speedLR", speedBL);
        // SmartDashboard.putNumber("CurAngle BL", backLeft.getSteerEncDeg());
        // SmartDashboard.putNumber("angleRR", angleBR);
        // SmartDashboard.putNumber("speedRR", speedBR);
        // SmartDashboard.putNumber("CurAngle BR", backRight.getSteerEncDeg());
        // SmartDashboard.putNumber("SpeedLF/MaxSpeed", speedFL / maxSpeed);
        if (deadStick) {

            // frontLeft.setSteerSpeed(0);
            swerve1.setDriveSpeed(0);
            // backLeft.setSteerSpeed(0);
            swerve2.setDriveSpeed(0);
            // frontRight.setSteerSpeed(0);
            swerve3.setDriveSpeed(0);
            // backRight.setSteerSpeed(0);
            swerve4.setDriveSpeed(0);

        } else {

            // Set each swerve module, scaling the drive speeds by the maximum speed
            swerve1.setSwerve(angleFL, speedFL / maxSpeed, this._driveCorrect);
            swerve2.setSwerve(angleBL, speedBL / maxSpeed, this._driveCorrect);
            swerve3.setSwerve(angleFR, speedFR / maxSpeed, this._driveCorrect);
            swerve4.setSwerve(angleBR, speedBR / maxSpeed, this._driveCorrect);
        }
        // this.FL_Drive.setAngleAndSpeed(angleLF, speedLF / maxSpeed);
        // this.BL_Drive.setAngleAndSpeed(angleLR, speedLR / maxSpeed);
        // this.FR_Drive.setAngleAndSpeed(angleRF, speedRF / maxSpeed);
        // this.BR_Drive.setAngleAndSpeed(angleRR, speedRR / maxSpeed);
        //getSteerEncoderVal();
    }

    private double speed(double val1, double val2) {
        return Math.sqrt((val1 * val1) + (val2 * val2));
    }

    private double angle(double val1, double val2) {
        return (Math.atan2(val2, val1)/Math.PI+0.5)/2;
    }

    public double[] getDriveEncoders() {
        double[] values = new double[] {
                swerve1.getDriveEncoder(),
                swerve2.getDriveEncoder(),
                swerve3.getDriveEncoder(),
                swerve4.getDriveEncoder()
        };

        return values;
    }

    public double getDriveEncoderAvg() {
        // double driveFL = frontLeft.getDriveEncoder();
        // double driveBL = backLeft.getDriveEncoder();
        // double driveFR = frontRight.getDriveEncoder();
        // double driveBR = backRight.getDriveEncoder();
        double driveFL = Math.abs(swerve1.getDriveEncoder());
        double driveBL = Math.abs(swerve2.getDriveEncoder());
        double driveFR = Math.abs(swerve3.getDriveEncoder());
        double driveBR = Math.abs(swerve4.getDriveEncoder());
        return (driveFL + driveFR + driveBL + driveBR) / 4.0;
    }

    public void setDriveEncodersPosition(double position) {
        swerve1.setDriveEncoder(position);
        swerve2.setDriveEncoder(position);
        swerve3.setDriveEncoder(position);
        swerve4.setDriveEncoder(position);
    }

    public void getSteerEncoderVal(){
        SmartDashboard.putNumber("angleLF", swerve1.getSteerEncoder());
        SmartDashboard.putNumber("angleLB", swerve2.getSteerEncoder());
        SmartDashboard.putNumber("angleRF", swerve3.getSteerEncoder());
        SmartDashboard.putNumber("angleRB", swerve4.getSteerEncoder());
    }

    // public static double[] getEncoderVal() {
    // 	double[] values = new double[] { frontLeft.getAnalogIn(), backLeft.getAnalogIn(), frontRight.getAnalogIn(),
    // 			backRight.getAnalogIn() };

    // 	return values;
    //}

    @Override()
    public void periodic() {

        odometer.update(this._gyro.getRotation2d(), getModulePositions());
        SmartDashboard.putNumber("Robot Heading", this._gyro.getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        //******** */
        // Uncomment following line to physically reset encoders position to zero state.
        // getSteerEncoderVal();

        //SmartDashboard.putNumber("Angle Back Left", backLeft.getSteerEncoder());
        // this.FL_Drive.outputToDashboard();
        // this.FR_Drive.outputToDashboard();
        // this.BR_Drive.outputToDashboard();
        // this.BL_Drive.outputToDashboard();
    }

    public void stopModules() {
        swerve1.stop();
        swerve3.stop();
        swerve2.stop();
        swerve4.stop();
    }

    public void disableRamping(){
        swerve1.driveMotorRamp(false);
        swerve3.driveMotorRamp(false);
        swerve2.driveMotorRamp(false);
        swerve4.driveMotorRamp(false);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.kPhysicalMaxSpeedMetersPerSecond/2);
        swerve1.setDesiredState(desiredStates[0]);
        swerve3.setDesiredState(desiredStates[1]);
        swerve2.setDesiredState(desiredStates[2]);
        swerve4.setDesiredState(desiredStates[3]);
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            swerve1.getPosition(),
            swerve3.getPosition(),
            swerve2.getPosition(),
            swerve4.getPosition()
        };
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            swerve1.getState(),
            swerve3.getState(),
            swerve2.getState(),
            swerve4.getState()
        };
    }
}
