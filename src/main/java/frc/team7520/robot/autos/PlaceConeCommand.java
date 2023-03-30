package frc.team7520.robot.autos;

import com.fasterxml.jackson.databind.type.ResolvedRecursiveType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team7520.robot.subsystems.Hand;

import static frc.team7520.robot.subsystems.Arm.armMotor;
import static frc.team7520.robot.subsystems.Arm.elbowMotor;


public class PlaceConeCommand extends CommandBase {

    boolean finished;

    public PlaceConeCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements();
    }

    @Override
    public void initialize() {
        finished = false;
    }

    @Override
    public void execute() {


        SparkMaxPIDController armPID = armMotor.getPIDController();
        SparkMaxPIDController elbowPID = elbowMotor.getPIDController();

        RelativeEncoder armEncoder = armMotor.getEncoder();
        RelativeEncoder elbowEncoder = elbowMotor.getEncoder();

        if (armEncoder.getPosition() < -100 && armEncoder.getPosition() > -105 && elbowEncoder.getPosition() > 62 && elbowEncoder.getPosition() < 70){

            finished = true;
        }



    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return finished;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
