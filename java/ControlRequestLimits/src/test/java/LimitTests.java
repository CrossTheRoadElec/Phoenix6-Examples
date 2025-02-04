import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static edu.wpi.first.units.Units.*;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.parallel.ResourceAccessMode;
import org.junit.jupiter.api.parallel.ResourceLock;

@ResourceLock(value = "SimState", mode = ResourceAccessMode.READ_WRITE)
public class LimitTests {
    final double SET_DELTA = 0.01;
    final int CONFIG_RETRY_COUNT = 5;

    TalonFX talonfx;

    private enum DriveDirection {
        Zero,
        Forward,
        Reverse
    }

    private class ControlRequestTest<T extends ControlRequest> {
        public final T request;
        public Consumer<DriveDirection> direction = null;
        public Consumer<Boolean> limitForward, limitReverse = null;
        public ControlRequestTest(T request) {
            this.request = request;
        }
    }

    ControlRequestTest<?>[] tests = new ControlRequestTest<?>[]
    {
        new ControlRequestTest<DutyCycleOut>(new DutyCycleOut(0)) {{
            direction = dir -> {
                if (dir == DriveDirection.Zero) request.Output = 0;
                if (dir == DriveDirection.Forward) request.Output = 1;
                if (dir == DriveDirection.Reverse) request.Output = -1;
            };
            limitForward = request::withLimitForwardMotion;
            limitReverse = request::withLimitReverseMotion;
        }},
        new ControlRequestTest<PositionDutyCycle>(new PositionDutyCycle(0)) {{
            direction = dir -> {
                if (dir == DriveDirection.Zero) request.Position = 0;
                if (dir == DriveDirection.Forward) request.Position = 1;
                if (dir == DriveDirection.Reverse) request.Position = -1;
            };
            limitForward = request::withLimitForwardMotion;
            limitReverse = request::withLimitReverseMotion;
        }},
        new ControlRequestTest<VelocityDutyCycle>(new VelocityDutyCycle(0)) {{
            direction = dir -> {
                if (dir == DriveDirection.Zero) request.Velocity = 0;
                if (dir == DriveDirection.Forward) request.Velocity = 1;
                if (dir == DriveDirection.Reverse) request.Velocity = -1;
            };
            limitForward = request::withLimitForwardMotion;
            limitReverse = request::withLimitReverseMotion;
        }},
        new ControlRequestTest<MotionMagicDutyCycle>(new MotionMagicDutyCycle(0)) {{
            direction = dir -> {
                if (dir == DriveDirection.Zero) request.Position = 0;
                if (dir == DriveDirection.Forward) request.Position = 1;
                if (dir == DriveDirection.Reverse) request.Position = -1;
            };
            limitForward = request::withLimitForwardMotion;
            limitReverse = request::withLimitReverseMotion;
        }},
        new ControlRequestTest<DynamicMotionMagicDutyCycle>(new DynamicMotionMagicDutyCycle(0, 0, 0, 0)) {{
            direction = dir -> {
                if (dir == DriveDirection.Zero) {
                    request.Position = 0;
                    request.Velocity = 10000;
                    request.Acceleration = 10000;
                    request.Jerk = 0;
                }
                if (dir == DriveDirection.Forward) {
                    request.Position = 1;
                    request.Velocity = 10000;
                    request.Acceleration = 10000;
                    request.Jerk = 0;
                }
                if (dir == DriveDirection.Reverse) {
                    request.Position = -1;
                    request.Velocity = 10000;
                    request.Acceleration = 10000;
                    request.Jerk = 0;
                }
            };
            limitForward = request::withLimitForwardMotion;
            limitReverse = request::withLimitReverseMotion;
        }},
        new ControlRequestTest<VoltageOut>(new VoltageOut(0)) {{
            direction = dir -> {
                if (dir == DriveDirection.Zero) request.Output = 0;
                if (dir == DriveDirection.Forward) request.Output = 1;
                if (dir == DriveDirection.Reverse) request.Output = -1;
            };
            limitForward = request::withLimitForwardMotion;
            limitReverse = request::withLimitReverseMotion;
        }},
        new ControlRequestTest<PositionVoltage>(new PositionVoltage(0)) {{
            direction = dir -> {
                if (dir == DriveDirection.Zero) request.Position = 0;
                if (dir == DriveDirection.Forward) request.Position = 1;
                if (dir == DriveDirection.Reverse) request.Position = -1;
            };
            limitForward = request::withLimitForwardMotion;
            limitReverse = request::withLimitReverseMotion;
        }},
        new ControlRequestTest<VelocityVoltage>(new VelocityVoltage(0)) {{
            direction = dir -> {
                if (dir == DriveDirection.Zero) request.Velocity = 0;
                if (dir == DriveDirection.Forward) request.Velocity = 1;
                if (dir == DriveDirection.Reverse) request.Velocity = -1;
            };
            limitForward = request::withLimitForwardMotion;
            limitReverse = request::withLimitReverseMotion;
        }},
        new ControlRequestTest<MotionMagicVoltage>(new MotionMagicVoltage(0)) {{
            direction = dir -> {
                if (dir == DriveDirection.Zero) request.Position = 0;
                if (dir == DriveDirection.Forward) request.Position = 1;
                if (dir == DriveDirection.Reverse) request.Position = -1;
            };
            limitForward = request::withLimitForwardMotion;
            limitReverse = request::withLimitReverseMotion;
        }},
        new ControlRequestTest<DynamicMotionMagicVoltage>(new DynamicMotionMagicVoltage(0, 0, 0, 0)) {{
            direction = dir -> {
                if (dir == DriveDirection.Zero) {
                    request.Position = 0;
                    request.Velocity = 10000;
                    request.Acceleration = 10000;
                    request.Jerk = 0;
                }
                if (dir == DriveDirection.Forward) {
                    request.Position = 1;
                    request.Velocity = 10000;
                    request.Acceleration = 10000;
                    request.Jerk = 0;
                }
                if (dir == DriveDirection.Reverse) {
                    request.Position = -1;
                    request.Velocity = 10000;
                    request.Acceleration = 10000;
                    request.Jerk = 0;
                }
            };
            limitForward = request::withLimitForwardMotion;
            limitReverse = request::withLimitReverseMotion;
        }},
        new ControlRequestTest<TorqueCurrentFOC>(new TorqueCurrentFOC(0)) {{
            direction = dir -> {
                if (dir == DriveDirection.Zero) request.Output = 0;
                if (dir == DriveDirection.Forward) request.Output = 1;
                if (dir == DriveDirection.Reverse) request.Output = -1;
            };
            limitForward = request::withLimitForwardMotion;
            limitReverse = request::withLimitReverseMotion;
        }},
        new ControlRequestTest<PositionTorqueCurrentFOC>(new PositionTorqueCurrentFOC(0)) {{
            direction = dir -> {
                if (dir == DriveDirection.Zero) request.Position = 0;
                if (dir == DriveDirection.Forward) request.Position = 1;
                if (dir == DriveDirection.Reverse) request.Position = -1;
            };
            limitForward = request::withLimitForwardMotion;
            limitReverse = request::withLimitReverseMotion;
        }},
        new ControlRequestTest<VelocityTorqueCurrentFOC>(new VelocityTorqueCurrentFOC(0)) {{
            direction = dir -> {
                if (dir == DriveDirection.Zero) request.Velocity = 0;
                if (dir == DriveDirection.Forward) request.Velocity = 1;
                if (dir == DriveDirection.Reverse) request.Velocity = -1;
            };
            limitForward = request::withLimitForwardMotion;
            limitReverse = request::withLimitReverseMotion;
        }},
        new ControlRequestTest<MotionMagicTorqueCurrentFOC>(new MotionMagicTorqueCurrentFOC(0)) {{
            direction = dir -> {
                if (dir == DriveDirection.Zero) request.Position = 0;
                if (dir == DriveDirection.Forward) request.Position = 1;
                if (dir == DriveDirection.Reverse) request.Position = -1;
            };
            limitForward = request::withLimitForwardMotion;
            limitReverse = request::withLimitReverseMotion;
        }},
        new ControlRequestTest<DynamicMotionMagicTorqueCurrentFOC>(new DynamicMotionMagicTorqueCurrentFOC(0, 0, 0, 0)) {{
            direction = dir -> {
                if (dir == DriveDirection.Zero) {
                    request.Position = 0;
                    request.Velocity = 10000;
                    request.Acceleration = 10000;
                    request.Jerk = 0;
                }
                if (dir == DriveDirection.Forward) {
                    request.Position = 1;
                    request.Velocity = 10000;
                    request.Acceleration = 10000;
                    request.Jerk = 0;
                }
                if (dir == DriveDirection.Reverse) {
                    request.Position = -1;
                    request.Velocity = 10000;
                    request.Acceleration = 10000;
                    request.Jerk = 0;
                }
            };
            limitForward = request::withLimitForwardMotion;
            limitReverse = request::withLimitReverseMotion;
        }},
    };

    @BeforeEach
    public void constructDevices() {
        assert HAL.initialize(500, 0);

        talonfx = new TalonFX(0);

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.Slot0.kP = 1; // Have some kP in slot 0 so we can drive with forward/reverse pos/vel
        cfg.MotionMagic.MotionMagicAcceleration = 10000; // Very large number to get a change in output quickly
        cfg.MotionMagic.MotionMagicCruiseVelocity = 10000; // Very large number to get a change in output quickly
        retryConfigApply(() -> talonfx.getConfigurator().apply(cfg));
        retryConfigApply(() -> talonfx.setPosition(0));

        /* enable the robot */
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();

        /* delay ~100ms so the devices can start up and enable */
        Timer.delay(0.100);
    }

    @Test
    public void robotIsEnabled() {
        assertTrue(DriverStation.isEnabled());
    }

    @Test
    public void testAllControlLimits() {
        for (var test : tests) {
            testControl(test);
        }
    }

    private void retryConfigApply(Supplier<StatusCode> toApply) {
        StatusCode finalCode = StatusCode.StatusCodeNotInitialized;
        int triesLeftOver = CONFIG_RETRY_COUNT;
        do {
            finalCode = toApply.get();
        } while (!finalCode.isOK() && --triesLeftOver > 0);
        assert(finalCode.isOK());
    }

    private <T extends ControlRequest> void testControl(ControlRequestTest<T> test) {
        for (var dir : DriveDirection.values()) {
            testControlPriv(test, dir, false, false);
            testControlPriv(test, dir, false, true);
            testControlPriv(test, dir, true, false);
            testControlPriv(test, dir, true, true);
        }
    }
    private <T extends ControlRequest> void testControlPriv(ControlRequestTest<T> test, DriveDirection direction, boolean forwardLimit, boolean reverseLimit) {
        var appliedMotorVoltage = talonfx.getMotorVoltage();
        var forwardLimitStatus = talonfx.getFault_ForwardHardLimit();
        var reverseLimitStatus = talonfx.getFault_ReverseHardLimit();

        test.limitForward.accept(forwardLimit);
        test.limitReverse.accept(reverseLimit);
        test.direction.accept(direction);

        talonfx.setControl(test.request);

        for (int i = 0; i < 20; ++i) {
            talonfx.getSimState().setRawRotorPosition(0); // Set position/velocity so we can drive in the desired direction
            talonfx.getSimState().setRotorVelocity(0);
            Timer.delay(0.02); /* Delay just enough for things like motion magic to ramp */
        }
        BaseStatusSignal.waitForAll(1, appliedMotorVoltage, forwardLimitStatus, reverseLimitStatus);

        System.out.println("");
        System.out.println("Testing Control request " + test.request.getName());
        System.out.println("Testing Direction " + direction.toString());
        System.out.println("Applied out is " + appliedMotorVoltage);
        System.out.println("Forward Limit is " + forwardLimit + " with fault " + forwardLimitStatus);
        System.out.println("Reverse Limit is " + reverseLimit + " with fault " + reverseLimitStatus);
        switch(direction) {
            case Zero:
                assertEquals(appliedMotorVoltage.getValue().in(Volts), 0, SET_DELTA);
                assertFalse(forwardLimitStatus.getValue());
                assertFalse(reverseLimitStatus.getValue());
                break;
            case Forward:
                if (forwardLimit) {
                    assertEquals(appliedMotorVoltage.getValue().in(Volts), 0, SET_DELTA);
                } else {
                    assertTrue(appliedMotorVoltage.getValue().in(Volts) > SET_DELTA);
                }
                assertTrue(forwardLimitStatus.getValue() == forwardLimit);
                assertFalse(reverseLimitStatus.getValue());
                break;
            case Reverse:
                if (reverseLimit) {
                    assertEquals(appliedMotorVoltage.getValue().in(Volts), 0, SET_DELTA);
                } else {
                    assertTrue(appliedMotorVoltage.getValue().in(Volts) < -SET_DELTA);
                }
                assertTrue(reverseLimitStatus.getValue() == reverseLimit);
                assertFalse(forwardLimitStatus.getValue());
                break;
        }
    }
}