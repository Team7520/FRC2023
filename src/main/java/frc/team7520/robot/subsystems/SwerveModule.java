// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team7520.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import org.opencv.core.Mat;

import java.lang.reflect.Array;
import java.util.ArrayList;

import static frc.team7520.robot.Constants.OperatorConstants.*;


public class SwerveModule extends PIDSubsystem
{

    public int steerId = 0;
    public int driveId = 0;
    public int encoderIdA = 0;
    public int encoderIdB = 0;
    public Encoder steerEncoder;
    public VictorSPX steerMotor;
    public CANSparkMax driveMotor;

    public SwerveModule(int steerId, int driveId, int encoderIdA, int encoderIdB) {
        super(new PIDController(.005, 0.001, 0.001));
        // Initialize the motor controllers
        // Initialize the encoders
        this.steerId = steerId;
        this.driveId = driveId;
        this.encoderIdA = encoderIdA;
        this.encoderIdB = encoderIdA;
        this.steerEncoder = new Encoder(encoderIdA, encoderIdB);
        this.steerMotor = new VictorSPX(steerId);
        this.driveMotor = new CANSparkMax(driveId, CANSparkMax.MotorType.kBrushless);
        steerEncoder.setDistancePerPulse(STEER_DISTANCE_PER_PULSE);
        getController().enableContinuousInput(0, 360);
        getController().setIntegratorRange(-1, 1);
    }


    /**
     * Example command factory method.
     *
     * @return a command
     */
    public void turnToAngle(Rotation2d angle)
    {
        double errorMargin = 10;
        double kP = 0.003;

        SmartDashboard.putNumber("Angle", steerEncoder.getDistance());
        SmartDashboard.putNumber("Angle Error", Math.abs(getController().getPositionError()));
        SmartDashboard.putNumber("modified error",  Math.abs(Math.abs(getController().getPositionError()) - 180));
        SmartDashboard.putBoolean("mod check", Math.abs(Math.abs(getController().getPositionError()) - 180) <= 10);
        SmartDashboard.putBoolean("check", Math.abs(getController().getPositionError()) <= 10);

        Rotation2d currentPosition = Rotation2d.fromDegrees(steerEncoder.getDistance()); // Get current encoder angle
        double error = angle.minus(currentPosition).getDegrees();

        double speed = MathUtil.clamp(getController().calculate(steerEncoder.getDistance(), angle.getDegrees()), -0.6, 0.6);/*error * kP*/;

        if(Math.abs(Math.abs(getController().getPositionError()) - 180) <= errorMargin /*|| Math.abs(getController().getPositionError()) <= errorMargin*/) {
            this.steerMotor.set(ControlMode.PercentOutput, 0);
        } else {
            this.steerMotor.set(ControlMode.PercentOutput, speed);
        }
    }

    public void drive(double speed)
    {
        this.driveMotor.set(speed*0.1);
    }

    public void resetEncoder()
    {
        this.steerEncoder.reset();
    }


    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
    }

    @Override
    protected void useOutput(double output, double setpoint) {

    }

    @Override
    protected double getMeasurement() {
        return this.steerEncoder.getDistance();
    }


    @Override
    public void simulationPeriodic()
    {
        // This method will be called once per scheduler run during simulation
    }
}
