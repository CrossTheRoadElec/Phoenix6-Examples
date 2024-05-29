package frc.robot;

import com.ctre.phoenix6.BaseStatusSignal;
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
  double HEIGHT = 1; //Controls the height of the mech2d SmartDashboard
  double WIDTH = 1; //Controls the height of the mech2d SmartDashboard
  double FX = 0;
  double TALON = 0;
  double CAN = 0;


  Mechanism2d mech = new Mechanism2d(WIDTH, HEIGHT);
  /* rotor rotor Ligaments */
  MechanismLigament2d rotorArm = mech.
                                getRoot("rotorpivotPoint", 0.25, 0.75).
                                append(new MechanismLigament2d("rotorArm", .1, 0, 0, new Color8Bit(Color.kAliceBlue)));

  MechanismLigament2d rotorSide1 = rotorArm.append(new MechanismLigament2d("rotorSide1", 0.076535, 112.5, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d rotorSide2 = rotorSide1.append(new MechanismLigament2d("rotorSide2", 0.076535, 45, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d rotorSide3 = rotorSide2.append(new MechanismLigament2d("rotorSide3", 0.076535, 45, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d rotorSide4 = rotorSide3.append(new MechanismLigament2d("rotorSide4", 0.076535, 45, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d rotorSide5 = rotorSide4.append(new MechanismLigament2d("rotorSide5", 0.076535, 45, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d rotorSide6 = rotorSide5.append(new MechanismLigament2d("rotorSide6", 0.076535, 45, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d rotorSide7 = rotorSide6.append(new MechanismLigament2d("rotorSide7", 0.076535, 45, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d rotorSide8 = rotorSide7.append(new MechanismLigament2d("rotorSide8", 0.076535, 45, 6, new Color8Bit(Color.kAliceBlue)));
  /* CANcoder ligaments */
  MechanismLigament2d ccArm = mech.
                                getRoot("CANpivotPoint", 0.25, 0.25).
                                append(new MechanismLigament2d("ccArm", .1, 0, 0, new Color8Bit(Color.kAliceBlue)));

  MechanismLigament2d ccSide1 = ccArm.append(new MechanismLigament2d("ccSide1", 0.076535, 112.5, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d ccSide2 = ccSide1.append(new MechanismLigament2d("ccSide2", 0.076535, 45, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d ccSide3 = ccSide2.append(new MechanismLigament2d("ccSide3", 0.076535, 45, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d ccSide4 = ccSide3.append(new MechanismLigament2d("ccSide4", 0.076535, 45, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d ccSide5 = ccSide4.append(new MechanismLigament2d("ccSide5", 0.076535, 45, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d ccSide6 = ccSide5.append(new MechanismLigament2d("ccSide6", 0.076535, 45, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d ccSide7 = ccSide6.append(new MechanismLigament2d("ccSide7", 0.076535, 45, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d ccSide8 = ccSide7.append(new MechanismLigament2d("ccSide8", 0.076535, 45, 6, new Color8Bit(Color.kAliceBlue)));
  /* FX Mechanism Ligaments */
  MechanismLigament2d mechanismArm = mech.
                                getRoot("TALONpivotPoint", 0.75, 0.5).
                                append(new MechanismLigament2d("mechanismArm", .2, 0, 0, new Color8Bit(Color.kAliceBlue)));

  MechanismLigament2d mechanismSide1 = mechanismArm.append(new MechanismLigament2d("mechanismSide1", 0.15307, 112.5, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d mechanismSide2 = mechanismSide1.append(new MechanismLigament2d("mechanismSide2", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d mechanismSide3 = mechanismSide2.append(new MechanismLigament2d("mechanismSide2", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d mechanismSide4 = mechanismSide3.append(new MechanismLigament2d("mechanismSide2", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d mechanismSide5 = mechanismSide4.append(new MechanismLigament2d("mechanismSide2", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d mechanismSide6 = mechanismSide5.append(new MechanismLigament2d("mechanismSide2", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d mechanismSide7 = mechanismSide6.append(new MechanismLigament2d("mechanismSide2", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d mechanismSide8 = mechanismSide7.append(new MechanismLigament2d("mechanismSide2", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));


  public void update(StatusSignal<Double> fxRotorPosition, StatusSignal<Double> fxPosition, StatusSignal<Double> cancoderPosition) {
    BaseStatusSignal.refreshAll(fxRotorPosition, fxPosition, cancoderPosition);
    rotorArm.setAngle(fxRotorPosition.getValue() * 360);
    mechanismArm.setAngle(fxPosition.getValue() * 360);
    ccArm.setAngle(cancoderPosition.getValue() * 360);
    SmartDashboard.putData("mech2d", mech); // Creates mech2d in SmartDashboard
  }
}
