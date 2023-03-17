import static org.junit.jupiter.api.Assertions.assertEquals;

import com.ctre.phoenixpro.BaseStatusSignalValue;
import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;

import edu.wpi.first.hal.HAL;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class FusedCANcoderTests {
    final double SET_DELTA = 0.1;

    TalonFX talon;
    CANcoder cancoder;

    @BeforeEach
    public void constructDevices() {
        assert HAL.initialize(500, 0);

        talon = new TalonFX(0);
        cancoder = new CANcoder(0);
    }

    @Test
    public void testIndividualPos() {
        final double TALON_POSITION = 0.4;
        final double CANCODER_POSITION = -3.1;

        /* Factory-default Talon and CANcoder */
        talon.getConfigurator().apply(new TalonFXConfiguration());
        cancoder.getConfigurator().apply(new CANcoderConfiguration());

        /* Get sim states */
        var talonSimState = talon.getSimState();
        var cancoderSimState = cancoder.getSimState();

        /* Wait for signal to update and assert they match the set positions */
        var talonPos = talon.getPosition();
        var cancoderPos = cancoder.getPosition();

        /* Make sure both are initially set to 0 before messing with sim state */
        talonSimState.setRawRotorPosition(0);
        cancoderSimState.setRawPosition(0);
        talon.setRotorPosition(0);
        cancoder.setPosition(0);
        /* Wait for sets to take affect */
        BaseStatusSignalValue.waitForAll(1.0, talonPos, cancoderPos);

        /* Set them to different values */
        talonSimState.setRawRotorPosition(TALON_POSITION);
        cancoderSimState.setRawPosition(CANCODER_POSITION);
        
        BaseStatusSignalValue.waitForAll(1.0, talonPos, cancoderPos);

        System.out.println("Talon Pos vs expected: " + talonPos + " vs " + TALON_POSITION);
        System.out.println("CANcoder Pos vs expected: " + cancoderPos + " vs " + CANCODER_POSITION);
        assertEquals(talonPos.getValue(), TALON_POSITION, SET_DELTA);
        assertEquals(cancoderPos.getValue(), CANCODER_POSITION, SET_DELTA);
    }

    @Test
    public void testFusedCANcoderPos() {
        final double TALON_POSITION = 0.4;
        final double CANCODER_POSITION = -3.1;

        /* Configure Talon to use Fused CANcoder, factory default CANcoder */
        var configs = new TalonFXConfiguration();
        configs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        configs.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
        talon.getConfigurator().apply(configs);
        cancoder.getConfigurator().apply(new CANcoderConfiguration());

        /* Get sim states */
        var talonSimState = talon.getSimState();
        var cancoderSimState = cancoder.getSimState();

        /* Wait for signal to update and assert they match the set positions */
        var talonPos = talon.getPosition();
        var cancoderPos = cancoder.getPosition();

        /* Make sure both are initially set to 0 before messing with sim state */
        talonSimState.setRawRotorPosition(0);
        cancoderSimState.setRawPosition(0);
        talon.setRotorPosition(0);
        cancoder.setPosition(0);
        /* Wait for sets to take affect */
        BaseStatusSignalValue.waitForAll(1.0, talonPos, cancoderPos);

        /* Set them to different values */
        talonSimState.setRawRotorPosition(TALON_POSITION);
        cancoderSimState.setRawPosition(CANCODER_POSITION);
        
        BaseStatusSignalValue.waitForAll(1.0, talonPos, cancoderPos);

        /* Further wait for Talon, since it probably just received the new CANcoder position frame */
        talonPos.waitForUpdate(1.0);

        /* Make sure Talon matches CANcoder, since it should be using CANcoder's position */
        System.out.println("Talon Pos vs expected: " + talonPos + " vs " + CANCODER_POSITION);
        System.out.println("CANcoder Pos vs expected: " + cancoderPos + " vs " + CANCODER_POSITION);
        assertEquals(talonPos.getValue(), CANCODER_POSITION, SET_DELTA);
        assertEquals(cancoderPos.getValue(), CANCODER_POSITION, SET_DELTA);
    }
}