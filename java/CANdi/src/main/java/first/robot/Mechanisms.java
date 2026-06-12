package first.robot;

import com.ctre.phoenix6.StatusSignal;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.smartdashboard.Mechanism2d;
import org.wpilib.smartdashboard.MechanismLigament2d;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.units.measure.Angle;
import org.wpilib.util.Color;
import org.wpilib.util.Color8Bit;

/**
 * Class to keep all the mechanism-specific objects together and out of the main example
 */
public class Mechanisms {
  private final double HEIGHT = 1;
  private final double WIDTH = 1;
  private final double ROOT_X = WIDTH / 2;
  private final double ROOT_Y = HEIGHT / 2;

  private Mechanism2d mech = new Mechanism2d(WIDTH, HEIGHT); // Main mechanism object
  private MechanismLigament2d wrist = mech.getRoot("base", ROOT_X, ROOT_Y)
      .append(new MechanismLigament2d("Wrist", .25, 90, 6, new Color8Bit(Color.ALICE_BLUE)));

  @SuppressWarnings("unused")
  private MechanismLigament2d leftArrow = wrist.append(
                                              new MechanismLigament2d("LeftArrow", 
                                                                      0.1, 
                                                                      150, 
                                                                      6, 
                                                                      new Color8Bit(Color.ALICE_BLUE)));
  @SuppressWarnings("unused")
  private MechanismLigament2d rightArrow = wrist.append(
                                               new MechanismLigament2d("RightArrow", 
                                                                       0.1, 
                                                                       -150, 
                                                                       6, 
                                                                       new Color8Bit(Color.ALICE_BLUE)));

  public void update(StatusSignal<Angle> angle) {
    SmartDashboard.putData("mech2d", mech); // Creates a mech2d window in GUI
    wrist.setAngle(new Rotation2d(angle.getValue())); // Converts 1 rotation to 360 degrees
  }
}
