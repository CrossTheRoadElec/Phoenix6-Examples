import static org.junit.jupiter.api.Assertions.assertEquals;

import com.ctre.phoenixpro.hardware.CANcoder;
import edu.wpi.first.hal.HAL;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class CANcoderTest {
    final double SET_DELTA = 0.1;

    CANcoder cancoder;

    @BeforeEach
    public void constructDevices() {
        assert HAL.initialize(500, 0);

        cancoder = new CANcoder(0);
    }

    @Test
    public void testPosSetter() {
        final double firstSet = 0;
        final double secondSet = 24;
        var posGetter = cancoder.getPosition();
        var cfg = cancoder.getConfigurator();

        cfg.setPosition(firstSet);
        assertEquals(posGetter.waitForUpdate(1).getValue(), firstSet, SET_DELTA);
        cfg.setPosition(secondSet);
        assertEquals(posGetter.waitForUpdate(1).getValue(), secondSet, SET_DELTA);
    }
}