import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Milliamps;
import static edu.wpi.first.units.Units.Rotations;
import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.S1StateValue;
import com.ctre.phoenix6.signals.S2CloseStateValue;
import com.ctre.phoenix6.signals.S2StateValue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.units.measure.Angle;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.parallel.ResourceAccessMode;
import org.junit.jupiter.api.parallel.ResourceLock;

@ResourceLock(value = "SimState", mode = ResourceAccessMode.READ_WRITE)
public class CANdiTests {
        final double SET_DELTA = 0.1;
    final int CONFIG_RETRY_COUNT = 5;

    private class SStateElement {
        private S1CloseStateValue assertState;
        private S1StateValue pinState;
        private boolean expectedValue;

        public SStateElement(S1CloseStateValue aS, S1StateValue pS, boolean ex) {
            assertState = aS;
            pinState = pS;
            expectedValue = ex;
        }
        public S1CloseStateValue closeState1() { return assertState; }
        public S1StateValue pinState1() { return pinState; }
        public S2CloseStateValue closeState2() { return S2CloseStateValue.valueOf(assertState.value); }
        public S2StateValue pinState2() { return S2StateValue.valueOf(pinState.value); }
        public boolean expectedValue() { return expectedValue; }
    }

    CANdi candi;

    @BeforeEach
    public void constructDevices() {
        assert HAL.initialize(500, 0);

        candi = new CANdi(0);
    }

    @Test
    public void testFloatStates() {
        var candiSimState = candi.getSimState();

        /* Factory-default CANdi */
        retryConfigApply(()->candi.getConfigurator().apply(new CANdiConfiguration()));

        /* Double check all permutations make sense */
        var permuations = new SStateElement[] { 
            new SStateElement(S1CloseStateValue.CloseWhenNotFloating, S1StateValue.Floating, false),
            new SStateElement(S1CloseStateValue.CloseWhenNotFloating, S1StateValue.High, true),
            new SStateElement(S1CloseStateValue.CloseWhenNotFloating, S1StateValue.Low, true),
            new SStateElement(S1CloseStateValue.CloseWhenFloating, S1StateValue.Floating, true),
            new SStateElement(S1CloseStateValue.CloseWhenFloating, S1StateValue.High, false),
            new SStateElement(S1CloseStateValue.CloseWhenFloating, S1StateValue.Low, false),
            new SStateElement(S1CloseStateValue.CloseWhenNotHigh, S1StateValue.Floating, true),
            new SStateElement(S1CloseStateValue.CloseWhenNotHigh, S1StateValue.High, false),
            new SStateElement(S1CloseStateValue.CloseWhenNotHigh, S1StateValue.Low, true),
            new SStateElement(S1CloseStateValue.CloseWhenHigh, S1StateValue.Floating, false),
            new SStateElement(S1CloseStateValue.CloseWhenHigh, S1StateValue.High, true),
            new SStateElement(S1CloseStateValue.CloseWhenHigh, S1StateValue.Low, false),
            new SStateElement(S1CloseStateValue.CloseWhenNotLow, S1StateValue.Floating, true),
            new SStateElement(S1CloseStateValue.CloseWhenNotLow, S1StateValue.High, true),
            new SStateElement(S1CloseStateValue.CloseWhenNotLow, S1StateValue.Low, false),
            new SStateElement(S1CloseStateValue.CloseWhenLow, S1StateValue.Floating, false),
            new SStateElement(S1CloseStateValue.CloseWhenLow, S1StateValue.High, false),
            new SStateElement(S1CloseStateValue.CloseWhenLow, S1StateValue.Low, true),
        };

        var s1State = candi.getS1State();
        var s1Closed = candi.getS1Closed();

        for(var e : permuations) {
            /* First configure close state value */
            var cfg = new CANdiConfiguration();
            cfg.DigitalInputs.S1CloseState = e.closeState1();
            retryConfigApply(()->candi.getConfigurator().apply(cfg));
            candiSimState.setS1State(e.pinState1());

            BaseStatusSignal.waitForAll(1, s1State, s1Closed);
            BaseStatusSignal.waitForAll(1, s1State, s1Closed);
            System.out.println("Configured for " + e.closeState1() + " and state is " + s1State + " and is closed: " + s1Closed);
            assertEquals(e.pinState1(), s1State.getValue());
            assertEquals(e.expectedValue(), s1Closed.getValue());
        }

        var s2State = candi.getS2State();
        var s2Closed = candi.getS2Closed();
        for(var e : permuations) {
            /* First pnfigure close state value */
            var cfg = new CANdiConfiguration();
            cfg.DigitalInputs.S2CloseState = e.closeState2();
            retryConfigApply(()->candi.getConfigurator().apply(cfg));
            candiSimState.setS2State(e.pinState2());

            BaseStatusSignal.waitForAll(1, s2State, s2Closed);
            BaseStatusSignal.waitForAll(1, s2State, s2Closed);
            System.out.println("Configured for " + e.closeState2() + " and state is " + s2State + " and is closed: " + s2Closed);
            assertEquals(e.pinState2(), s2State.getValue());
            assertEquals(e.expectedValue(), s2Closed.getValue());
        }
    }

    @Test
    public void overCurrentTest() {
        retryConfigApply(()->candi.getConfigurator().apply(new CANdiConfiguration()));

        var candiSimState = candi.getSimState();
        var isOvercurrent = candi.getOvercurrent();
        var outputCurrent = candi.getOutputCurrent();

        BaseStatusSignal.waitForAll(1, isOvercurrent, outputCurrent);

        isOvercurrent.waitForUpdate(1);

        System.out.println("CANdi output current is " + outputCurrent + " and is overcurrent: " + isOvercurrent);
        assertEquals(isOvercurrent.getValue(), false);
        assertEquals(outputCurrent.getValue().in(Amps), 0, SET_DELTA);

        /* CANdi can provide up to 300 mA */
        candiSimState.setOutputCurrent(Milliamps.of(250));

        BaseStatusSignal.waitForAll(1, isOvercurrent, outputCurrent);

        System.out.println("CANdi output current is " + outputCurrent + " and is overcurrent: " + isOvercurrent);
        assertEquals(isOvercurrent.getValue(), false);
        assertEquals(outputCurrent.getValue().in(Milliamps), 250, SET_DELTA);

        /* CANdi will fault over 300 mA */
        candiSimState.setOutputCurrent(Amps.of(1));

        BaseStatusSignal.waitForAll(1, isOvercurrent, outputCurrent);

        System.out.println("CANdi output current is " + outputCurrent + " and is overcurrent: " + isOvercurrent);
        assertEquals(isOvercurrent.getValue(), true);
        assertEquals(outputCurrent.getValue().in(Amps), 0, SET_DELTA);
    }

    @Test
    public void testIndividualPos() {
        final Angle CANDI_POSITION_1 = Rotations.of(-3.1);
        final Angle CANDI_POSITION_2 = Rotations.of(1.2);
        final Angle CANDI_POSITION_3 = Rotations.of(0.7);

        /* Factory-default CANdi */
        retryConfigApply(()->candi.getConfigurator().apply(new CANdiConfiguration()));

        /* Get sim states */
        var candiSimState = candi.getSimState();

        /* Wait for signal to update and assert they match the set positions */
        var candiPos1 = candi.getPWM1Position();
        var candiPos2 = candi.getPWM2Position();
        var candiPos3 = candi.getQuadraturePosition();

        /* Make sure both are initially set to 0 before messing with sim state */
        retryConfigApply(()->candiSimState.setPwm1Position(Rotations.of(0)));
        retryConfigApply(()->candiSimState.setPwm2Position(Rotations.of(0)));
        retryConfigApply(()->candiSimState.setRawQuadraturePosition(Rotations.of(0)));
        retryConfigApply(()->candi.setQuadraturePosition(Rotations.of(0)));
        /* Wait for sets to take affect */
        BaseStatusSignal.waitForAll(1.0, candiPos1, candiPos2, candiPos3);

        /* Set them to different values */
        retryConfigApply(()->candiSimState.setPwm1Position(CANDI_POSITION_1));
        retryConfigApply(()->candiSimState.setPwm2Position(CANDI_POSITION_2));
        retryConfigApply(()->candiSimState.setRawQuadraturePosition(CANDI_POSITION_3));

        BaseStatusSignal.waitForAll(1.0, candiPos1, candiPos2, candiPos3);
        BaseStatusSignal.waitForAll(1.0, candiPos1, candiPos2, candiPos3);

        System.out.println("CANdi Pos vs expected: " + candiPos1 + " vs " + CANDI_POSITION_1);
        assertEquals(candiPos1.getValue().in(Rotations), CANDI_POSITION_1.in(Rotations), SET_DELTA);
        System.out.println("CANdi Pos vs expected: " + candiPos2 + " vs " + CANDI_POSITION_2);
        assertEquals(candiPos2.getValue().in(Rotations), CANDI_POSITION_2.in(Rotations), SET_DELTA);
        System.out.println("CANdi Pos vs expected: " + candiPos3 + " vs " + CANDI_POSITION_3);
        assertEquals(candiPos3.getValue().in(Rotations), CANDI_POSITION_3.in(Rotations), SET_DELTA);
    }


    private void retryConfigApply(Supplier<StatusCode> toApply) {
        StatusCode finalCode = StatusCode.StatusCodeNotInitialized;
        int triesLeftOver = CONFIG_RETRY_COUNT;
        do{
            finalCode = toApply.get();
        } while (!finalCode.isOK() && --triesLeftOver > 0);
        assert(finalCode.isOK());
    }
}
