package frc.robot.sim;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.sim.PhysicsSim.SimProfile;

/**
 * Holds information about a simulated TalonFX.
 */
class TalonFXSimProfile extends SimProfile {
    private final double kMotorResistance = 0.002; // Assume 2mOhm resistance for voltage drop calculation
    private final TalonFXSimState _falconSim;
    private final DCMotorSim _motorSim;

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
    public TalonFXSimProfile(final TalonFX falcon, final double rotorInertia) {
        this._motorSim = new DCMotorSim(DCMotor.getFalcon500Foc(1), 1.0, rotorInertia);
        this._falconSim = falcon.getSimState();
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

        _motorSim.setInputVoltage(_falconSim.getMotorVoltage());

        _motorSim.update(getPeriod());

        /// SET SIM PHYSICS INPUTS
        final double position_rot = _motorSim.getAngularPositionRotations();
        final double velocity_rps = Units.radiansToRotations(_motorSim.getAngularVelocityRadPerSec());

        _falconSim.setRawRotorPosition(position_rot);
        _falconSim.setRotorVelocity(velocity_rps);

        _falconSim.setSupplyVoltage(12 - _falconSim.getSupplyCurrent() * kMotorResistance);
    }
}