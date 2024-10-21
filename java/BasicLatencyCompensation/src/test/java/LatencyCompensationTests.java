import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class LatencyCompensationTests {
    final double DOUBLE_DELTA = 0.01;

    TalonFX talonfx;
    CANcoder cancoder;

    @BeforeEach
    public void constructDevices() {
        assert HAL.initialize(500, 0);

        talonfx = new TalonFX(0);
        cancoder = new CANcoder(0);
    }
    
    @Test
    public void testLatencyCompensator() {
        final Angle position = Rotations.of(35);
        final AngularVelocity velocity = RotationsPerSecond.of(24);
        /* Initialize by making all the positions 0 */
        talonfx.setPosition(0);
        cancoder.setPosition(0);

        /* Set the simulated state of device positions */
        talonfx.getSimState().setRawRotorPosition(position);
        talonfx.getSimState().setRotorVelocity(velocity);
        cancoder.getSimState().setRawPosition(position);
        cancoder.getSimState().setVelocity(velocity);

        /* Perform latency compensation */
        /* Start by getting signals */
        var talonPos = talonfx.getPosition();
        var talonVel = talonfx.getVelocity();
        var cancoderPos = cancoder.getPosition();
        var cancoderVel = cancoder.getVelocity();

        /* Wait for an update on all of them so they're synchronized */
        StatusCode status = StatusCode.OK;
        for(int i = 0; i < 5; ++i)
        {
            System.out.println("Waiting on signals");
            status = BaseStatusSignal.waitForAll(1, talonPos, talonVel, cancoderPos, cancoderVel);
            if(status.isOK()) break;
        }
        assertTrue(status.isOK());

        /* Wait a bit longer for the latency to actually do some work */
        try {
            Thread.sleep(10);
        }catch(Exception ex) {}
        /* Calculate how much latency we'd expect */
        Time talonLatency = Seconds.of(talonPos.getTimestamp().getLatency());
        Angle compensatedTalonPos = position.plus(velocity.times(talonLatency));
        Time cancoderLatency = Seconds.of(cancoderPos.getTimestamp().getLatency());
        Angle compensatedCANcoderPos = position.plus(velocity.times(cancoderLatency));

        /* Calculate compensated values before the assert to avoid timing issue related to it */
        var functionCompensatedTalon = BaseStatusSignal.getLatencyCompensatedValue(talonPos, talonVel);
        var functionCompensatedCANcoder = BaseStatusSignal.getLatencyCompensatedValue(cancoderPos, cancoderVel);

        /* Assert the two methods match */
        System.out.println("Talon Pos: " + compensatedTalonPos + " - " + functionCompensatedTalon);
        System.out.println("CANcoder Pos: " + compensatedCANcoderPos + " - " + functionCompensatedCANcoder);
        assertEquals(compensatedTalonPos.in(Rotations), functionCompensatedTalon.in(Rotations), DOUBLE_DELTA);
        assertEquals(compensatedCANcoderPos.in(Rotations), functionCompensatedCANcoder.in(Rotations), DOUBLE_DELTA);
    }
}
