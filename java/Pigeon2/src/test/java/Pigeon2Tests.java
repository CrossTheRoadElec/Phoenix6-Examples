import static org.junit.jupiter.api.Assertions.assertEquals;

import com.ctre.phoenixpro.hardware.Pigeon2;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Timer;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class Pigeon2Tests {
    final double SET_DELTA = 0.1;

    Pigeon2 pidgey;

    @BeforeEach
    public void constructDevices() {
        assert HAL.initialize(500, 0);

        pidgey = new Pigeon2(0);
    }

    @Test
    public void testYawSetter() {
        final double firstSet = 0;
        final double secondSet = 24;
        var yawGetter = pidgey.getYaw();
        var cfg = pidgey.getConfigurator();

        /* First make simulation yaw 0 */
        var simState= pidgey.getSimState();
        simState.setRawYaw(0);

        cfg.setYaw(firstSet);
        yawGetter.waitForUpdate(1);
        System.out.println("First yaw is " + firstSet + " vs Pidgey's " + yawGetter.getValue());
        assertEquals(yawGetter.getValue(), firstSet, SET_DELTA);
        cfg.setYaw(secondSet);
        yawGetter.waitForUpdate(1);
        System.out.println("Second yaw is " + secondSet + " vs Pidgey's " + yawGetter.getValue());
        assertEquals(yawGetter.getValue(), secondSet, SET_DELTA);
    }

    @Test
    public void testSimulationYawSetter() {
        final double firstSet = 0;
        final double secondSet = 24;
        var yawGetter = pidgey.getYaw();
        var cfg = pidgey.getConfigurator();

        /* First make simulation yaw 0 */
        var simState= pidgey.getSimState();
        simState.setRawYaw(0);
        cfg.setYaw(0);

        simState.setRawYaw(firstSet);
        yawGetter.waitForUpdate(1);
        System.out.println("First yaw is " + firstSet + " vs Pidgey's " + yawGetter.getValue());
        assertEquals(yawGetter.getValue(), firstSet, SET_DELTA);
        simState.setRawYaw(secondSet);
        yawGetter.waitForUpdate(1);
        System.out.println("Second yaw is " + secondSet + " vs Pidgey's " + yawGetter.getValue());
        assertEquals(yawGetter.getValue(), secondSet, SET_DELTA);
    }

    @Test
    public void testSimulationRotation2d() {
        final double firstSet = 0;
        final double secondSet = 24;
        var cfg = pidgey.getConfigurator();

        /* First make simulation yaw 0 */
        var simState= pidgey.getSimState();
        simState.setRawYaw(0);
        cfg.setYaw(0);

        simState.setRawYaw(firstSet);
        Timer.delay(0.1); // There is no synchronous getRotation2d, so we instead have to delay
        var rotation = pidgey.getRotation2d();
        System.out.println("First yaw is " + firstSet + " vs rotation 2d's " + rotation.getDegrees());
        assertEquals(rotation.getDegrees(), firstSet, SET_DELTA);

        simState.setRawYaw(secondSet);
        Timer.delay(0.1); // There is no synchronous getRotation2d, so we instead have to delay
        rotation = pidgey.getRotation2d();
        System.out.println("Second yaw is " + secondSet + " vs rotation 2d's " + rotation.getDegrees());
        assertEquals(rotation.getDegrees(), secondSet, SET_DELTA);
    }
}