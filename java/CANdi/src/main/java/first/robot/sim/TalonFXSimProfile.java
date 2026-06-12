package first.robot.sim;

import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.DCMotorSim;

import first.robot.sim.PhysicsSim.SimProfile;

/**
 * Holds information about a simulated TalonFX.
 */
class TalonFXSimProfile extends SimProfile {
    private static final double kMotorResistance = 0.002; // Assume 2mOhm resistance for voltage drop calculation
    private final TalonFXSimState _talonFXSim;
    private final CANdi _candi;
    private final double _gearRatio;

    private final DCMotorSim _motorSim;

    /**
     * Creates a new simulation profile for a TalonFX device.
     *
     * @param talonFX
     *                        The TalonFX device
     * @param candi
     *                        The CANdi associated with the TalonFX
     * @param gearRatio
     *                        The gear ratio from the TalonFX to the mechanism
     * @param rotorInertia
     *                        Rotational Inertia of the mechanism at the rotor
     */
    public TalonFXSimProfile(final TalonFX talonFX, final CANdi candi, final double gearRatio, final double rotorInertia) {
        this._talonFXSim = talonFX.getSimState();
        this._candi = candi;
        this._gearRatio = gearRatio;

        var gearbox = DCMotor.getKrakenX60Foc(1);
        this._motorSim = new DCMotorSim(Models.singleJointedArmFromPhysicalConstants(gearbox, rotorInertia, gearRatio), gearbox);
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
        _motorSim.setInputVoltage(_talonFXSim.getMotorVoltage());

        _motorSim.update(getPeriod());

        // SET SIM PHYSICS INPUTS
        final var position = Radians.of(_motorSim.getAngularPosition() * _gearRatio);
        final var velocity = RadiansPerSecond.of(_motorSim.getAngularVelocity() * _gearRatio);

        _talonFXSim.setRawRotorPosition(position);
        _talonFXSim.setRotorVelocity(velocity);

        _talonFXSim.setSupplyVoltage(12 - _talonFXSim.getSupplyCurrent() * kMotorResistance);


        _candi.getSimState().setPwm1Position(position.div(_gearRatio));
        _candi.getSimState().setPwm1Velocity(velocity.div(_gearRatio));
    }
}