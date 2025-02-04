import static org.junit.jupiter.api.Assertions.assertTrue;
import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.parallel.ResourceAccessMode;
import org.junit.jupiter.api.parallel.ResourceLock;

@ResourceLock(value = "SimState", mode = ResourceAccessMode.READ_WRITE)
public class CurrentLimitTests implements AutoCloseable {
    final int CONFIG_RETRY_COUNT = 5;

    TalonFX talon;

    @Override
    public void close() {
        /* destroy our TalonFX object */
        talon.close();
    }

    @BeforeEach
    public void constructDevices() {
        assert HAL.initialize(500, 0);

        talon = new TalonFX(0);

        /* enable the robot */
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();

        /* delay ~100ms so the devices can start up and enable */
        Timer.delay(0.100);
    }

    @AfterEach
    void shutdown() {
        close();
    }

    @Test
    public void robotIsEnabled() {
        /* verify that the robot is enabled */
        assertTrue(DriverStation.isEnabled());
    }

    @Test
    public void testStatorLimit() {
        var statorCurrent = talon.getStatorCurrent();

        /* Configure a stator limit of 20 amps */
        TalonFXConfiguration toConfigure = new TalonFXConfiguration();
        CurrentLimitsConfigs currentLimitConfigs = toConfigure.CurrentLimits;
        currentLimitConfigs.StatorCurrentLimit = 20;
        currentLimitConfigs.StatorCurrentLimitEnable = false; // Start with stator limits off

        retryConfigApply(() -> talon.getConfigurator().apply(toConfigure));

        /* Put the talon in a stall, which should produce a lot of current */
        talon.setControl(new DutyCycleOut(1));
        /* wait for the control to apply */
        Timer.delay(0.020);

        /* Get the next update for stator current */
        statorCurrent.waitForUpdate(1);

        System.out.println("Stator current is " + statorCurrent);
        assertTrue(statorCurrent.getValue().in(Amps) > 100); // Stator current should be in excess of 100 amps

        /* Now apply the stator current limit */
        currentLimitConfigs.StatorCurrentLimitEnable = true;
        retryConfigApply(() -> talon.getConfigurator().apply(currentLimitConfigs));

        /* wait for the current to drop */
        Timer.delay(0.500);

        /* Get the next update for stator current */
        statorCurrent.waitForUpdate(1);

        System.out.println("Stator current is " + statorCurrent);
        assertTrue(statorCurrent.getValue().in(Amps) < 25); // Give some wiggle room
    }

    @Test
    public void testSupplyLimit() {
        var supplyCurrent = talon.getSupplyCurrent();

        /* Configure a supply limit of 10 amps */
        TalonFXConfiguration toConfigure = new TalonFXConfiguration();
        CurrentLimitsConfigs currentLimitConfigs = toConfigure.CurrentLimits;
        currentLimitConfigs.SupplyCurrentLowerLimit = 5;
        currentLimitConfigs.SupplyCurrentLimit = 40;
        currentLimitConfigs.SupplyCurrentLowerTime = 1.0;
        currentLimitConfigs.StatorCurrentLimitEnable = false;
        currentLimitConfigs.SupplyCurrentLimitEnable = false; // Start with supply limits off

        retryConfigApply(() -> talon.getConfigurator().apply(toConfigure));

        /* Put the talon in a stall, which should produce a lot of current */
        talon.setControl(new DutyCycleOut(1));
        /* wait for the control to apply */
        Timer.delay(0.100);

        /* Get the next update for supply current */
        supplyCurrent.waitForUpdate(1);

        System.out.println("Supply current is " + supplyCurrent);
        assertTrue(supplyCurrent.getValue().in(Amps) > 80); // Supply current should be in excess of 80 amps

        /* Now apply the supply current limit */
        currentLimitConfigs.SupplyCurrentLimitEnable = true;
        retryConfigApply(() -> talon.getConfigurator().apply(currentLimitConfigs));

        /* wait for the current to adjust */
        Timer.delay(0.500);

        /* Get the next update for supply current */
        supplyCurrent.waitForUpdate(1);

        System.out.println("Supply current is " + supplyCurrent);
        assertTrue(supplyCurrent.getValue().in(Amps) < 50); // Make sure it's limited to 40 amps (time hasn't exceeded 1 second in total)
        assertTrue(supplyCurrent.getValue().in(Amps) > 30); // Give some wiggle room

        /* Wait a full extra couple seconds so the limit kicks in and starts limiting us */
        Timer.delay(2);

        /* Get the next update for supply current */
        supplyCurrent.waitForUpdate(1);

        System.out.println("Supply current is " + supplyCurrent);
        assertTrue(supplyCurrent.getValue().in(Amps) < 10); // Give some wiggle room
    }

    private void retryConfigApply(Supplier<StatusCode> toApply) {
        StatusCode finalCode = StatusCode.StatusCodeNotInitialized;
        int triesLeftOver = CONFIG_RETRY_COUNT;
        do {
            finalCode = toApply.get();
        } while (!finalCode.isOK() && --triesLeftOver > 0);
        assert(finalCode.isOK());
    }
}