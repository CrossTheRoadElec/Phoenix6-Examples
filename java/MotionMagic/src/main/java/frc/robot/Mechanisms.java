package frc.robot;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * Class to keep all the mechanism-specific objects together and out of the main example
 */
public class Mechanisms {
    double HEIGHT = 1; // Controls the height of the mech2d SmartDashboard
    double WIDTH = 1; // Controls the height of the mech2d SmartDashboard
  
    Mechanism2d mech = new Mechanism2d(WIDTH, HEIGHT);
    // Velocity
    MechanismLigament2d VelocityMech = mech.
                                getRoot("velocityLineReferencePosition", 0.75, 0.5).
                                append(new MechanismLigament2d("velocityLine",  1,90, 6, new Color8Bit(Color.kAliceBlue)));
    
   MechanismLigament2d midline = mech.
                                 getRoot("midline", 0.7, 0.5).
                                 append(new MechanismLigament2d("midline", 0.1, 0, 3, new Color8Bit(Color.kCyan)));
                           
   //Position                            
   MechanismLigament2d arm = mech.
                                 getRoot("pivotPoint", 0.25, 0.5).
                                 append(new MechanismLigament2d("arm", .2, 0, 0, new Color8Bit(Color.kAliceBlue)));
 
   MechanismLigament2d side1 = arm.append(new MechanismLigament2d("side1", 0.15307, 112.5, 6, new Color8Bit(Color.kAliceBlue)));
   MechanismLigament2d side2 = side1.append(new MechanismLigament2d("side2", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
   MechanismLigament2d side3 = side2.append(new MechanismLigament2d("side3", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
   MechanismLigament2d side4 = side3.append(new MechanismLigament2d("side4", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
   MechanismLigament2d side5 = side4.append(new MechanismLigament2d("side5", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
   MechanismLigament2d side6 = side5.append(new MechanismLigament2d("side6", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
   MechanismLigament2d side7 = side6.append(new MechanismLigament2d("side7", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
   MechanismLigament2d side8 = side7.append(new MechanismLigament2d("side8", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));

    /**
     * Runs the mech2d widget in GUI.
     *  
     * This utilizes GUI to simulate and display a TalonFX and exists to allow users to test and understand 
     * features of our products in simulation using our examples out of the box. Users may modify to have a 
     * display interface that they find more intuitive or visually appealing.
     */                            
    public void update(StatusSignal<Double> position, StatusSignal<Double> velocity) {  
        VelocityMech.setLength(velocity.getValue()/120); // Divide by 120 to scale motion to fit in the window
        arm.setAngle(position.getValue() * 360);
        SmartDashboard.putData("mech2d", mech); // Creates mech2d in SmartDashboard
    }                             
}
