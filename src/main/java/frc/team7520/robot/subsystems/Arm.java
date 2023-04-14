package frc.team7520.robot.subsystems;

import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.team7520.robot.RobotContainer;
import static frc.team7520.robot.RobotContainer.*;

public class Arm extends SubsystemBase {

    public static final CANSparkMax armMotor = RobotContainer.armMotor;
    public static final CANSparkMax elbowMotor = RobotContainer.elbowMotor;
    public static final DigitalInput input = RobotContainer.input;
    public static final DigitalInput photoSwitch = RobotContainer.photoSwitch;
    private final SparkMaxPIDController armPID;
    private final SparkMaxPIDController elbowPID;


    private RelativeEncoder elbowEncoder;
    private RelativeEncoder armEncoder;

    Position currentPosition = Position.REST;

    enum Position {
        FLOOR(0, 28.6),
        CUBE(-87, 69),
        CONE(-100, 65),
        DUNK(-103, 70),
        REST(0,0);

        public double arm;
        public double elbow;
        Position(double arm, double elbow) {
            this.arm = arm;
            this.elbow = elbow;
        }
    }



    // With eager singleton initialization, any static variables/fields used in the
    // constructor must appear before the "INSTANCE" variable so that they are initialized
    // before the constructor is called when the "INSTANCE" variable initializes.

    /**
     * The Singleton instance of this Hand. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static Arm INSTANCE = new Arm();

    /**
     * Returns the Singleton instance of this Hand. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code Hand.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static Arm getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this Hand. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private Arm() {
        this.elbowEncoder = elbowMotor.getEncoder();
//        this.elbowEncoder.setPosition(0);
        this.armEncoder = armMotor.getEncoder();
//        this.armEncoder.setPosition(0);

        this.armPID = armMotor.getPIDController();
        this.elbowPID = elbowMotor.getPIDController();

        elbowPID.setOutputRange(-1,1);
        armPID.setOutputRange(-1,1);

        elbowPID.setP(1);
        elbowPID.setI(0);
        elbowPID.setD(0);
        elbowPID.setFF(.1);

        armPID.setP(1);
        armPID.setI(0);
        armPID.setD(0);
        armPID.setFF(0.1);

        elbowPID.setOutputRange(-0.5,0.25);
        armPID.setOutputRange(-1,1);

        elbowMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        armMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    }

    public Command dunk(){
        return runOnce( () -> {
            setPosition(Position.DUNK);
        });
    }

    public Command cube() {
        return runOnce( () -> {
            setPosition(Position.CUBE);
        }).andThen(moveArm());
    }

    public Command cone() {
        return runOnce( () -> {
            setPosition(Position.CONE);
        }).andThen(moveArm());
    }

    public Command floor(){
        return runOnce( () -> {
            setPosition(Position.FLOOR);
        });
    }

    public Command rest() {
        return runOnce( () -> {
            setPosition(Position.REST);
        }).andThen(moveArm());
    }

    boolean idleMode = false;

//    public Command toggle(){
//        return runOnce(() -> {
//            idleMode = !idleMode;
//
//            if(idleMode){
//                elbowMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
//                armMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
//            }else{
//                elbowMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
//                armMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
//            }
//        });
//    }

    // public boolean floor() {
    //     boolean complete;
    //     return complete;

    // }

//    public boolean calibrate() {
//        if (calibrated && calibrated_E) {
//            fullCalibration = true;
//          }
//
//          if(!fullCalibration) {
//
//            if (!calibrated) { //If shoulder is not callibrated
//              armMotor.set(0.035);
//              if(!input.get()) { //When upper arm hits the switch
//                armNeoEncoder.setPosition(SET_SHOULDER); //Set this position as base position, all calculations based here
//                calibrated = true;
//              }
//
//            } else {
//              armMotor.set(-0.001); //Freeze upper arm
//              if(!calibrated_E) {
//                elbowMotor.set(-0.022); //start calibrating lower arm
//                if(!photoSwitch.get()) { //When photoswitch detects a reflection
//                  elbowEncoder.setPosition(SET_EBLOW); //Set base position for lower
//                  calibrated_E = true;
//                }
//              }
//            }
//          }
//          return fullCalibration;
//    }

//    public Command manual(double leftStick, double rightStick) { //God mode, joy stick control. CHECK IF JOYSTICK IS INVERTED
//          return this.runOnce(() -> {
//              if(leftStick < -0.15) {  //If joystick holds UP
//                  if(shoulderPosition < 19) { //When arm bypasses the upper LIMIT zone, STOP
//                      armMotor.set(neo500rest);
//                  } else if (shoulderPosition < 29 && shoulderPosition > 19){ //WHen arm enters BUFFER zone, slow down
//                      armMotor.set(leftStick/8);
//                  } else { //If not in the buffer nor limit zone, move at a casual speed
//                      armMotor.set(leftStick/4);
//                  }
//
//              } else if(leftStick > 0.15) { //If joystick holds DOWN
//                  if (shoulderPosition > 140) { //When arm bypasses lower LIMIT zone, STOP
//                      armMotor.set(neo500rest);
//                  } else if (shoulderPosition < 140 && shoulderPosition > 130){ //When arm enters BUFFER zone, slow down
//                      armMotor.set(leftStick / 8);
//                  } else { //When neither in buffer nor limit zone, move at casual speed
//                      armMotor.set(leftStick / 4);
//                  }
//
//              } else { //If joystick is not touched, feedforward
//                  armMotor.set(neo550rest);
//              }
//
//              if(Math.abs(rightStick) > 0.15) { //for lower arm, there are NO LIMITE nor BUFFER zones
//                  elbowMotor.set(rightStick / 8);
//              } else {
//                  elbowMotor.set(neo550rest); //Feedforward lower arm
//              }
//          });
//    }

    public void setPosition(Position position){

        currentPosition = position;

    }

    public Command changePos(){

        return runOnce(() ->{

            double elbow = operatorController.getRightY();
            double arm = operatorController.getLeftY();

            System.out.println(Math.abs(arm));

            if(Math.abs(arm) > 0.05){
                System.out.println(arm);

                currentPosition.arm = currentPosition.arm + 1 * arm;
            }

            if(Math.abs(elbow) > 0.05){

                System.out.println(elbow);
                currentPosition.elbow =  currentPosition.elbow - 1 * elbow;
            }

        }).andThen(moveArm());

    }

    public Command moveArm() { //Auto positioning
        return runOnce(() -> {

            armPID.setReference(currentPosition.arm, CANSparkMax.ControlType.kPosition);
            elbowPID.setReference(currentPosition.elbow, CANSparkMax.ControlType.kPosition);
        });

    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("arm", armEncoder.getPosition());
        SmartDashboard.putNumber("Elbow", elbowEncoder.getPosition());



    }
}

