package frc.robot;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * Class to keep all the mechanism-specific objects together and out of the main example
 */
public class Mechanisms {
  private final double HEIGHT = 1;
  private final double WIDTH = 1;
  private final double ROOT_X = WIDTH / 2;
  private final double ROOT_Y = HEIGHT / 2;

  private final Pigeon2SimState pidgeySim; // We need a sim state in order to change the values of pidgey
  private final DCMotorSim motorSim = new DCMotorSim(DCMotor.getFalcon500(1), 100, 100);
  private final XboxController controller = new XboxController(0); // Uses an Xbox controller for setting the CANcoder simulation
  private Mechanism2d mech = new Mechanism2d(WIDTH, HEIGHT); // Main mechanism object
  private MechanismLigament2d wrist = mech.
                                      getRoot("base", ROOT_X, ROOT_Y).
                                      append(new MechanismLigament2d("Wrist", .25, 90, 6, new Color8Bit(Color.kAliceBlue)));

  @SuppressWarnings("unused")
  private MechanismLigament2d leftArrow = wrist.append(new MechanismLigament2d("LeftArrow", 0.1, 150, 6, new Color8Bit(Color.kAliceBlue)));
  @SuppressWarnings("unused")
  private MechanismLigament2d rightArrow = wrist.append(new MechanismLigament2d("RightArrow", 0.1, -150, 6, new Color8Bit(Color.kAliceBlue)));

  public Mechanisms(Pigeon2 pigeon) {
    pidgeySim = pigeon.getSimState();
  }

  public void update(StatusSignal<Double> position) {
    wrist.setAngle(position.getValue());
    SmartDashboard.putData("mech2d", mech);
  }

  /**
   * Didn't want to move in our full physics class, since this is just for pigeon
   * Call this periodically in the simulation periodic method
   */
  public void simUpdate(){
    double Yaxis = controller.getLeftY();
    double motorVoltage = Yaxis * 12; // scales joystick axis to motor voltage ( +-12v)
    motorSim.setInputVoltage(motorVoltage);
    motorSim.update(.02);
    double position = motorSim.getAngularPositionRotations()*360;
    pidgeySim.setRawYaw(position);
  }
}
