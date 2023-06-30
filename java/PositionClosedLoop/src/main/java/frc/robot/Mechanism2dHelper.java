package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * Holds information about the mech2d widget in GUI simulation.
 */
public class Mechanism2dHelper {
    private static final Mechanism2dHelper Instance = new Mechanism2dHelper();

    /**
     * Gets the robot simulator instance.
     */
    public static Mechanism2dHelper getInstance() {
        return Instance;
    }


    private final double HEIGHT = .5; // Controls the height of the mech2d SmartDashboard
    private final double WIDTH = 1; // Controls the height of the mech2d SmartDashboard

    Mechanism2d mech = new Mechanism2d(WIDTH, HEIGHT);
    MechanismLigament2d distanceBar = mech.
                                getRoot("startPoint", 0.5, 0.4).
                                append(new MechanismLigament2d("distanceBar",  1, 0, 6, new Color8Bit(Color.kAliceBlue)));
  
    MechanismLigament2d reference = mech.
                                getRoot("baseLine", 0, .1).
                                append(new MechanismLigament2d("baseLine", 1, 0, 6, new Color8Bit(Color.kCyan)));
  
    MechanismLigament2d joint = mech.
                                getRoot("neckLine", 0.5, .1).
                                append(new MechanismLigament2d("neckLine", 0.3, 90, 6, new Color8Bit(Color.kCyan)));

    /**
     * Runs the mech2d widget in GUI.
     *  
     * This utilizes GUI to simulate and display a TalonFX and exists to allow users to test and understand 
     * features of our products in simulation using our examples out of the box. Users may modify to have a 
     * display interface that they find more intuitive or visually appealing.
     */                            
    public void distanceBarSetLength(TalonFX m_fx) {
        distanceBar.setLength(m_fx.getPosition().getValue()/30); // Divide by 30 to scale motion to fit in the window 
        SmartDashboard.putData("mech2d", mech); // Creates mech2d in SmartDashboard
    }                             
}
