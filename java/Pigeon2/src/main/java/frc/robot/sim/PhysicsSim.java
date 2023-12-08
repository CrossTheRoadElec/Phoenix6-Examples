package frc.robot.sim;

import java.util.ArrayList;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

/**
 * Manages physics simulation for CTRE products.
 */
public class PhysicsSim {
    private static final PhysicsSim sim = new PhysicsSim();

    /**
     * Gets the robot simulator instance.
     */
    public static PhysicsSim getInstance() {
        return sim;
    }

    /**
     * Adds a TalonFX controller to the simulator.
     * 
     * @param falcon
     *        The TalonFX device
     * @param can
     *        The CANcoder device
     * @param gearRatio
     *        The gear reduction of the TalonFX
     * @param rotorInertia
     *        Rotational Inertia of the mechanism at the rotor
     */
    public void addTalonFX(TalonFX falcon, Pigeon2 pigeon, final double rotorInertia) {
        if (falcon != null) {
            TalonFXSimProfile simFalcon = new TalonFXSimProfile(falcon, pigeon, rotorInertia);
            _simProfiles.add(simFalcon);
        }
    }

    /**
     * Runs the simulator:
     * - enable the robot
     * - simulate sensors
     */
    public void run() {
        // Simulate devices
        for (SimProfile simProfile : _simProfiles) {
            simProfile.run();
        }
    }

    private final ArrayList<SimProfile> _simProfiles = new ArrayList<SimProfile>();

    /**
     * Holds information about a simulated device.
     */
    static class SimProfile {
        private double _lastTime;
        private boolean _running = false;

        /**
         * Runs the simulation profile.
         * Implemented by device-specific profiles.
         */
        public void run() {
        }

        /**
         * Returns the time since last call, in seconds.
         */
        protected double getPeriod() {
            // set the start time if not yet running
            if (!_running) {
                _lastTime = Utils.getCurrentTimeSeconds();
                _running = true;
            }

            double now = Utils.getCurrentTimeSeconds();
            final double period = now - _lastTime;
            _lastTime = now;

            return period;
        }
    }
}