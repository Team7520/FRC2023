package frc.team7520.robot.subsystems;


import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class Hand extends SubsystemBase {

    private Servo servo0, servo1, servo2, servo3;
    static double topFinger, bottomFinger;

    // With eager singleton initialization, any static variables/fields used in the
    // constructor must appear before the "INSTANCE" variable so that they are initialized
    // before the constructor is called when the "INSTANCE" variable initializes.

    /**
     * The Singleton instance of this Hand. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static Hand INSTANCE = new Hand();

    /**
     * Returns the Singleton instance of this Hand. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code Hand.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static Hand getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this Hand. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private Hand() {
        this.servo0 = new Servo(0);
        this.servo1 = new Servo(1);
        this.servo2 = new Servo(2);
        this.servo3 = new Servo(3);
    }

    public void openHand(){
        topFinger = 0.3 + 0.15;
        bottomFinger = 0.3 + 0.15;
        servo0.set(topFinger);
        servo1.set(1-topFinger);
        servo2.set(bottomFinger);
        servo3.set(1-bottomFinger);




    }

    public void closeHand(){

        topFinger =0.3;
        bottomFinger = 0.3;
        servo0.set(topFinger);
        servo1.set(1-topFinger);
        servo2.set(bottomFinger);
        servo3.set(1-bottomFinger);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("topFinger", topFinger);
        SmartDashboard.putNumber("bottomFinger", bottomFinger);
    }
}

