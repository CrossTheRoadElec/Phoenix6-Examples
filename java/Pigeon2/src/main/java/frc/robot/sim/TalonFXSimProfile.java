package frc.robot.sim;

import com.ctre.phoenix6.hardware.Pigeon2;
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
    private final TalonFXSimState _talonFXSim;
    private final Pigeon2 _pigeon;

    private final DCMotorSim _motorSim;

    /**
     * Creates a new simulation profile for a TalonFX device.
     * 
     * @param talonFX
     *                        The TalonFX device
     * @param pigeon
     *                        The Pigeon 2 associated with the TalonFX
     * @param rotorInertia
     *                        Rotational Inertia of the mechanism at the rotor
     */
    public TalonFXSimProfile(final TalonFX talonFX, final Pigeon2 pigeon, final double rotorInertia) {
        this._talonFXSim = talonFX.getSimState();
        this._pigeon = pigeon;
        var gearbox = DCMotor.getKrakenX60Foc(1);
        this._motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, rotorInertia, 1.0), gearbox);
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

        _pigeon.getSimState().setRawYaw(position_rot);

    }
}