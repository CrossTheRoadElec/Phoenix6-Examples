package frc.robot.sim;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.sim.PhysicsSim.SimProfile;

/**
 * Holds information about a simulated TalonFX.
 */
class TalonFXSimProfile extends SimProfile {
    private static final double kMotorResistance = 0.002; // Assume 2mOhm resistance for voltage drop calculation

    private final DCMotorSim _motorSim;
    private final TalonFXSimState _talonFXSim;

    /**
     * Creates a new simulation profile for a TalonFX device.
     * 
     * @param talonFX
     *                        The TalonFX device
     * @param rotorInertia
     *                        Rotational Inertia of the mechanism at the rotor
     */
    public TalonFXSimProfile(final TalonFX talonFX, final double rotorInertia) {
        var gearbox = DCMotor.getKrakenX60Foc(1);
        this._motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, rotorInertia, 1.0), gearbox);
        this._talonFXSim = talonFX.getSimState();
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

        _motorSim.setInputVoltage(_talonFXSim.getMotorVoltage());

        _motorSim.update(getPeriod());

        /// SET SIM PHYSICS INPUTS
        final double position_rot = _motorSim.getAngularPositionRotations();
        final double velocity_rps = Units.radiansToRotations(_motorSim.getAngularVelocityRadPerSec());

        _talonFXSim.setRawRotorPosition(position_rot);
        _talonFXSim.setRotorVelocity(velocity_rps);

        _talonFXSim.setSupplyVoltage(12 - _talonFXSim.getSupplyCurrent() * kMotorResistance);
    }
}