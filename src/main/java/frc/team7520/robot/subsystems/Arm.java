package frc.team7520.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.DigitalInput;
import java.util.*;
import frc.team7520.robot.RobotContainer;
import frc.team7520.robot.Constants;

public class Arm extends SubsystemBase {

    public static final CANSparkMax NeoMotor500 = RobotContainer.NeoMotor500;
    public static final CANSparkMax NeoMotor550 = RobotContainer.NeoMotor550;
    public static final DigitalInput input = RobotContainer.input;
    public static final DigitalInput photoSwitch = RobotContainer.photoSwitch;
    

    private RelativeEncoder m_encoder550;
    private RelativeEncoder m_encoder500;

    private static double speedMultiplierS = 1;
    private static double speedMultiplierE = 1;
    private static double neo500up = -0.2 * speedMultiplierS;
    private static double neo500down = 0.1 * speedMultiplierS;
    private static double neo550up = -0.15 * speedMultiplierE;
    private static double neo550down = 0.1 * speedMultiplierE;
    private static double neo500rest = -0.01;
    private static double neo550rest = -0.01;
    private static double shoulderSpeed;
    private static double shoulderPosition;
    private static double elbowPosition, elbowSpeed;
    private static final int SET_SHOULDER = 150;
    private static final int SET_EBLOW = 10;
    
    private static boolean calibrated = false;
    private static boolean calibrated_E = false;
    private static boolean fullCalibration = false;
    private static boolean armPlace;
    
    

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
        m_encoder550= NeoMotor550.getEncoder();
        m_encoder550.setPosition(0);
        m_encoder500= NeoMotor500.getEncoder();
        m_encoder500.setPosition(0);

    }

    public boolean cube() {
        boolean complete = setArmPosition(46, 64);
        return complete; //true = complete, false = incomplete
    }

    public boolean cone() {
        boolean complete = setArmPosition(25, 73);
        return complete;
    }

    public boolean rest() {
        boolean complete = setArmPosition(121, 26);
        return complete;
    }

    // public boolean floor() {
    //     boolean complete;
    //     return complete;

    // }

    public boolean calibrate() {
        if (calibrated && calibrated_E) { 
            fullCalibration = true;
          }

          if(!fullCalibration) { 

            if (!calibrated) { //If shoulder is not callibrated
              NeoMotor500.set(0.035);
              if(!input.get()) { //When upper arm hits the switch
                m_encoder500.setPosition(SET_SHOULDER); //Set this position as base position, all calculations based here
                calibrated = true;
              } 

            } else {
              NeoMotor500.set(-0.001); //Freeze upper arm
              if(!calibrated_E) {
                NeoMotor550.set(-0.022); //start calibrating lower arm
                if(!photoSwitch.get()) { //When photoswitch detects a reflection
                  m_encoder550.setPosition(SET_EBLOW); //Set base position for lower
                  calibrated_E = true;
                } 
              }
            }
          }
          return fullCalibration;
    }

    public void manual(double leftStick, double rightStick) { //God mode, joy stick control. CHECK IF JOYSTICK IS INVERTED
        if(leftStick < -0.15) {  //If joystick holds UP
            if(shoulderPosition < 19) { //When arm bypasses the upper LIMIT zone, STOP
              NeoMotor500.set(neo500rest); 
            } else if (shoulderPosition < 29 && shoulderPosition > 19){ //WHen arm enters BUFFER zone, slow down
              NeoMotor500.set(leftStick/8);
            } else { //If not in the buffer nor limit zone, move at a casual speed
              NeoMotor500.set(leftStick/4);
            }

          } else if(leftStick > 0.15) { //If joystick holds DOWN
            if (shoulderPosition > 140) { //When arm bypasses lower LIMIT zone, STOP
              NeoMotor500.set(neo500rest);
            } else if (shoulderPosition < 140 && shoulderPosition > 130){ //When arm enters BUFFER zone, slow down
              NeoMotor500.set(leftStick / 8);
            } else { //When neither in buffer nor limit zone, move at casual speed
              NeoMotor500.set(leftStick / 4); 
            }

          } else { //If joystick is not touched, feedforward
            NeoMotor500.set(neo550rest); 
          }

          if(Math.abs(rightStick) > 0.15) { //for lower arm, there are NO LIMITE nor BUFFER zones
            NeoMotor550.set(rightStick / 8);
          } else {
            NeoMotor550.set(neo550rest); //Feedforward lower arm
          }
    }

    public boolean setArmPosition(double upperArm, double lowerArm) { //Auto positioning
        double upperShoulderRange = upperArm+3; //The range of acceptable positions for upper
        double lowerShoulderRange = upperArm-3;
        double upperElbowRange = lowerArm+3; //The range of acceptable positions for lower
        double lowerElbowRange = lowerArm-3;
        double slowdownRange = 20; //Buffer zone range for slowing down
        

        if (Math.abs(shoulderPosition - upperArm) > slowdownRange){ //When outside buffer zone, fast speed
            speedMultiplierS = 2;
        } else { //If inside buffer, slow down
            speedMultiplierS = 1;
        }
        if (Math.abs(elbowPosition - lowerArm) > slowdownRange){ 
            speedMultiplierE = 2;
        } else { 
            speedMultiplierE = 1;
        }

        if (shoulderPosition < lowerShoulderRange) { //When above wanted value
            NeoMotor500.set(neo500down);
            armPlace = false;
        } else if (shoulderPosition > upperShoulderRange) { //when below wanted value
            NeoMotor500.set(neo500up);
            armPlace = false;
        } else { //when in position range
            NeoMotor500.set(neo500rest); 
            armPlace = true; //activate lower arm
        }

        if (armPlace) { //when big arm is position, lower arm
            if (elbowPosition < lowerElbowRange) {
                NeoMotor550.set(neo550down);
            } else if (elbowPosition > upperElbowRange) {
                NeoMotor550.set(neo550up);
            } else {
                NeoMotor550.set(neo550rest);
                return true;
                
            }

        } else { //before big arm is not positioned, bring lower arm up to prevent crashing into other objects
            if (elbowPosition < 23) {
                NeoMotor550.set(neo550down);
            } else if (elbowPosition > 29) {
                NeoMotor550.set(neo550up);
            } else {
                NeoMotor550.set(neo550rest);
                
            }
        }
        return false;
    
    }
    
    @Override
    public void periodic() {
        neo500up = -0.2 * speedMultiplierS;
        neo500down = 0.1 * speedMultiplierS;
        neo550up = -0.15 * speedMultiplierE;
        neo550down = 0.1 * speedMultiplierE;

        //SmartDashboard.putBoolean("Shoulder Calibrated", calibrated);
        SmartDashboard.putBoolean("Elbow Calibrated", calibrated_E); //before you drive, check for both booleans to be green
        SmartDashboard.putBoolean("Full Calibration", fullCalibration);
        SmartDashboard.putNumber("Neo500up Speed", neo500up);
        SmartDashboard.putNumber("Neo550up Speed", neo550up);
        SmartDashboard.putNumber("Neo500down Speed", neo500down);
        SmartDashboard.putNumber("Neo550down Speed", neo550down);


        elbowPosition = m_encoder550.getPosition();
        elbowSpeed = m_encoder550.getVelocity();
        SmartDashboard.putNumber("ForeArm Position", elbowPosition);
        SmartDashboard.putNumber("ForeArm Speed", elbowSpeed);

        shoulderPosition = m_encoder500.getPosition();
        shoulderSpeed = m_encoder500.getVelocity();
        SmartDashboard.putNumber("Arm Position", shoulderPosition);
        SmartDashboard.putNumber("Arm Speed", shoulderSpeed);

    }
}

