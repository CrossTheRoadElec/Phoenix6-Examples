package frc.robot.sim;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.sim.PhysicsSim.SimProfile;

/**
 * Holds information about a simulated TalonFX.
 */
class TalonFXSimProfile extends SimProfile {
    private final double kMotorResistance = 0.002; // Assume 2mOhm resistance for voltage drop calculation
    private final TalonFX _falcon;
    private final CANcoder _canCoder;
    private final double _gearRatio;

    private final DCMotorSim _motorSim;
    private final CANcoderSimState _cancoderSim; // We need a sim state in order to change the values of CANcoder
    private final XboxController m_controller = new XboxController(0); 

    /**
     * Creates a new simulation profile for a TalonFX device.
     * 
     * @param falcon
     *                        The TalonFX device
     * @param accelToFullTime
     *                        The time the motor takes to accelerate from 0 to full,
     *                        in seconds
     * @param fullVel
     *                        The maximum motor velocity, in rotations per second
     * @param sensorPhase
     *                        The phase of the TalonFX sensors
     */
    public TalonFXSimProfile(final TalonFX falcon, final CANcoder canCoder, final double gearRatio, final double rotorInertia) {
        this._falcon = falcon;
        this._canCoder = canCoder;
        this._gearRatio = gearRatio;
        this._motorSim = new DCMotorSim(DCMotor.getFalcon500(1), 100, .001);
        this._cancoderSim = _canCoder.getSimState();
    }

    /**
     * Runs the simulation profile.
     * 
     * This uses very rudimentary physics simulation and exists to allow users to
     * test features of our products in simulation using our examples out of the
     * box. Users may modify this to utilize more accurate physics simulation.
     */
    public void run() {
        // DEVICE SPEED SIMULATION
        _motorSim.setInputVoltage(_falcon.getSimState().getMotorVoltage());

        _motorSim.update(getPeriod());

        // SET SIM PHYSICS INPUTS
        double velocity_rps = Units.radiansToRotations(_motorSim.getAngularVelocityRadPerSec());

        _falcon.getSimState().setRawRotorPosition(_motorSim.getAngularPositionRotations());
        _falcon.getSimState().setRotorVelocity(velocity_rps);

        _falcon.getSimState().setSupplyVoltage(12 - _falcon.getSimState().getSupplyCurrent() * kMotorResistance);

        _canCoder.getSimState().setRawPosition(_motorSim.getAngularPositionRotations()/_gearRatio);
        _canCoder.getSimState().setVelocity(velocity_rps/_gearRatio);

        double Yaxis = m_controller.getLeftY();
        double motorVoltage = Yaxis * 12; // Scales joystick axcis to motor voltage ( +-12v)
        _motorSim.setInputVoltage(motorVoltage);
        _motorSim.update(.02);
        double position = _motorSim.getAngularPositionRotations();
        double velocity = _motorSim.getAngularVelocityRPM()/60;
        _cancoderSim.setRawPosition(position);
        _cancoderSim.setVelocity(velocity);

    }
}