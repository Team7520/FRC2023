package frc.team7520.robot.subsystems;


import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.team7520.robot.RobotContainer.compressor;

public class Hand extends SubsystemBase {

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

    boolean isCompressed = false;

    DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 8, 9);

    ColorSensorV3 colorSensorV3 = new ColorSensorV3(I2C.Port.kOnboard);

    /**
     * Creates a new instance of this Hand. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private Hand() {

    }

    public Command openHand(){
        return runOnce( () -> {
            solenoid.set(DoubleSolenoid.Value.kReverse);
        });
    }

    public Command closeHand(){
        return runOnce( () -> {
            solenoid.set(DoubleSolenoid.Value.kForward);
        });
    }

    public Command autoClose(){
        return runOnce( () -> {
            if (colorSensorV3.getProximity() > 2000){
                solenoid.set(DoubleSolenoid.Value.kForward);
            }
        });
    }

    @Override
    public void periodic() {

        if(compressor.getPressure() < 109 && !isCompressed){
            compressor.enableAnalog(110,111);
        }else {
            if(!isCompressed) {
                isCompressed = true;
                compressor.enableAnalog(70,80);
            };
        }

        SmartDashboard.putBoolean("e", isCompressed);

        SmartDashboard.putNumber("ee", compressor.getPressure());

        SmartDashboard.putBoolean("eee", compressor.getPressure() > 119);


    }
}

