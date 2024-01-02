package frc.robot.sim;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.sim.PhysicsSim.SimProfile;

/**
 * Holds information about a simulated TalonFX.
 */
class TalonFXSimProfile extends SimProfile {
    private static final double kMotorResistance = 0.002; // Assume 2mOhm resistance for voltage drop calculation
    private final TalonFX _falcon;
    private final Pigeon2 _pigeon;

    private final DCMotorSim _motorSim;

    /**
     * Creates a new simulation profile for a TalonFX device.
     * 
     * @param falcon
     *                        The TalonFX device
     * @param pigeon
     *                        The Pigeon 2 associated with the TalonFX
     * @param rotorInertia
     *                        Rotational Inertia of the mechanism at the rotor
     */
    public TalonFXSimProfile(final TalonFX falcon, final Pigeon2 pigeon, final double rotorInertia) {
        this._falcon = falcon;
        this._pigeon = pigeon;
        this._motorSim = new DCMotorSim(DCMotor.getFalcon500Foc(1), 1.0, rotorInertia);
    }

    /**
     * Runs the simulation profile.
     * 
     * This uses very rudimentary physics simulation and exists to allow users to
     * test features of our products in simulation using our examples out of the
     * box. Users may modify this to utilize more accurate physics simulation.
     */
    public void run() {
        /// DEVICE SPEED SIMULATION

        _motorSim.setInputVoltage(_falcon.getSimState().getMotorVoltage());

        _motorSim.update(getPeriod());

        /// SET SIM PHYSICS INPUTS
        final double position_rot = _motorSim.getAngularPositionRotations();
        final double velocity_rps = Units.radiansToRotations(_motorSim.getAngularVelocityRadPerSec());

        _falcon.getSimState().setRawRotorPosition(position_rot);
        _falcon.getSimState().setRotorVelocity(velocity_rps);

        _falcon.getSimState().setSupplyVoltage(12 - _falcon.getSimState().getSupplyCurrent() * kMotorResistance);

        _pigeon.getSimState().setRawYaw(position_rot);

    }
}