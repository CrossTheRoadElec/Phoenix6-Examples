import static org.junit.jupiter.api.Assertions.assertEquals;

import com.ctre.phoenixpro.hardware.Pigeon2;
import edu.wpi.first.hal.HAL;

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

        cfg.setYaw(firstSet);
        assertEquals(yawGetter.waitForUpdate(1).getValue(), firstSet, SET_DELTA);
        cfg.setYaw(secondSet);
        assertEquals(yawGetter.waitForUpdate(1).getValue(), secondSet, SET_DELTA);
    }
}