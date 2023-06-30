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
    private final TalonFX _falcon;
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
        this._falcon = new TalonFX(0);
        this._motorSim = new DCMotorSim(DCMotor.getFalcon500(1), 1.0, rotorInertia);
        this._falconSim = _falcon.getSimState();
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
        double velocity_rps = Units.radiansToRotations(_motorSim.getAngularVelocityRadPerSec());

        _falcon.getSimState().setRawRotorPosition(_motorSim.getAngularPositionRotations());
        _falcon.getSimState().setRotorVelocity(velocity_rps);

        _falcon.getSimState().setSupplyVoltage(12 - _falcon.getSimState().getSupplyCurrent() * kMotorResistance);

        double rotorPosition = _motorSim.getAngularPositionRotations() / 10; //Gear reduction by 10:1 so rotation is visible in sim
        _falconSim.setRawRotorPosition(rotorPosition);

        double rotorVelocity = _motorSim.getAngularVelocityRadPerSec(); // TODO move this to TalonFXSimProfile
        _falconSim.setRotorVelocity(rotorVelocity);
    }
}