import static org.junit.jupiter.api.Assertions.assertTrue;
import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.parallel.ResourceAccessMode;
import org.junit.jupiter.api.parallel.ResourceLock;

@ResourceLock(value = "SimState", mode = ResourceAccessMode.READ_WRITE)
public class GadgeteerTests implements AutoCloseable {
    final int CONFIG_RETRY_COUNT = 5;

    // Allow error of up to .01 rotation
    final double ALLOWABLE_DELTA = 0.01;

    TalonFXS talon;

    @Override
    public void close() {
        /* destroy our TalonFX object */
        talon.close();
    }

    @BeforeEach
    public void constructDevices() {
        assert HAL.initialize(500, 0);

        talon = new TalonFXS(0);

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
    public void testMotorPositionGetter() {
        /* Set the motor encoder to minion */
        TalonFXSConfiguration toConfigure = new TalonFXSConfiguration();
        toConfigure.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        retryConfigApply(() -> talon.getConfigurator().apply(toConfigure));

        StatusSignal<Angle> position = talon.getPosition();

        /* Set the position of the current selected encoder */
        talon.setPosition(Rotations.of(50));

        /* wait for the setter to apply */
        Timer.delay(0.020);

        position.refresh();

        System.out.println("Position " + position.getValueAsDouble());
        assertTrue(nearlyEqual(position.getValueAsDouble(), 50)); 
    }

    @Test
    public void testPulseWidthPosition() {
        testGadgeteerPosition(ExternalFeedbackSensorSourceValue.PulseWidth);
    }

    @Test
    public void testQuadraturePosition() {
        testGadgeteerPosition(ExternalFeedbackSensorSourceValue.Quadrature);
    }

    @Test
    public void testPulseWidthVelocity() {
        testGadgeteerVelocity(ExternalFeedbackSensorSourceValue.PulseWidth);
    }

    @Test
    public void testQuadratureVelocity() {
        testGadgeteerVelocity(ExternalFeedbackSensorSourceValue.Quadrature);
    }

    public void testGadgeteerVelocity(ExternalFeedbackSensorSourceValue feedbackType) {
        /* Set the motor encoder to minion */
        TalonFXSConfiguration toConfigure = new TalonFXSConfiguration();
        toConfigure.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        toConfigure.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.Commutation;

        retryConfigApply(() -> talon.getConfigurator().apply(toConfigure));

        StatusSignal<AngularVelocity> velocity = talon.getVelocity();
        StatusSignal<AngularVelocity> gadgeteerVelocity;

        /* raw quad and pwm setters are disabled by default */
        if (feedbackType == ExternalFeedbackSensorSourceValue.Quadrature) {
            gadgeteerVelocity = talon.getRawQuadratureVelocity();
        } else {
            gadgeteerVelocity = talon.getRawPulseWidthVelocity();
        }

        /* wait for the setter to apply */
        Timer.delay(0.020);

        if (feedbackType == ExternalFeedbackSensorSourceValue.Quadrature) {
            /* Set the velocity of the selected encoder */
            talon.getSimState().setQuadratureVelocity(RotationsPerSecond.of(50));
        } else {
            /* Set the velocity of the selected encoder */
            talon.getSimState().setPulseWidthVelocity(RotationsPerSecond.of(50));
        }

        /* Set the velocity of the rotor */
        talon.getSimState().setRotorVelocity(RotationsPerSecond.of(20));

        /* wait for the setter to apply */
        Timer.delay(0.020);

        gadgeteerVelocity.waitForUpdate(0.1);
        velocity.waitForUpdate(0.1);

        System.out.println("Gadgeteer Velocity " + gadgeteerVelocity.getValueAsDouble());
        assertTrue(nearlyEqual(gadgeteerVelocity.getValueAsDouble(), 50)); 

        /* Our feedback is currently defaulted to commutation, so this will be the value of the encoder */
        System.out.println("Velocity " + velocity.getValueAsDouble());
        assertTrue(nearlyEqual(velocity.getValueAsDouble(), 20));

        /* Set feedback, which should mean the selected encoder velocity and regular velocity are the same */
        toConfigure.ExternalFeedback.ExternalFeedbackSensorSource = feedbackType;
        retryConfigApply(() -> talon.getConfigurator().apply(toConfigure));

        /* Fetch updated information */
        gadgeteerVelocity.waitForUpdate(0.1);
        velocity.waitForUpdate(0.1);

        /* Encoder should be correct */
        System.out.println("Gadgeteer Velocity " + gadgeteerVelocity.getValueAsDouble());

        /* Our feedback is now the same as our encoder, so velocity should be the same */
        System.out.println("Velocity " + velocity.getValueAsDouble());

        assertTrue(nearlyEqual(velocity.getValueAsDouble(), gadgeteerVelocity.getValueAsDouble()));
    }

    public void testGadgeteerPosition(ExternalFeedbackSensorSourceValue feedbackType) {
        /* Set the motor encoder to minion */
        TalonFXSConfiguration toConfigure = new TalonFXSConfiguration();
        toConfigure.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        toConfigure.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.Commutation;

        retryConfigApply(() -> talon.getConfigurator().apply(toConfigure));

        StatusSignal<Angle> regularPosition = talon.getPosition();
        StatusSignal<Angle> gadgeteerPosition;

        /* raw quad and pwm setters are disabled by default */
        if (feedbackType == ExternalFeedbackSensorSourceValue.Quadrature) {
            gadgeteerPosition = talon.getRawQuadraturePosition();
        } else {
            gadgeteerPosition = talon.getRawPulseWidthPosition();
        }

        gadgeteerPosition.setUpdateFrequency(Hertz.of(250));

        /* wait for the setter to apply */
        Timer.delay(0.020);

        if (feedbackType == ExternalFeedbackSensorSourceValue.Quadrature) {
            /* Set the position of the attached pulse-width encoder */
            talon.getSimState().setRawQuadraturePosition(Rotations.of(50));
        } else {
            /* Set the position of the attached pulse-width encoder */
            talon.getSimState().setPulseWidthPosition(Rotations.of(50));
        }

        /* Set the position of the selected encoder */
        talon.setPosition(20);

        /* wait for the setter to apply */
        Timer.delay(0.020);

        gadgeteerPosition.waitForUpdate(0.1);
        regularPosition.waitForUpdate(0.1);

        System.out.println("Gadgeteer Position " + gadgeteerPosition.getValueAsDouble());
        assertTrue(nearlyEqual(gadgeteerPosition.getValueAsDouble(), 50)); 

        /* Our feedback is currently defaulted to commutation, so this will be the value of the encoder */
        System.out.println("Position " + regularPosition.getValueAsDouble());
        assertTrue(nearlyEqual(regularPosition.getValueAsDouble(), 20));

        /* Set feedback to our selected feedback type, this means that the raw getter and regular getter should match */
        toConfigure.ExternalFeedback.ExternalFeedbackSensorSource = feedbackType;
        retryConfigApply(() -> talon.getConfigurator().apply(toConfigure));

        /* Fetch updated information */
        gadgeteerPosition.waitForUpdate(0.1);
        regularPosition.waitForUpdate(0.1);

        /* Encoder position should be correct */
        System.out.println("Gadgeteer Position " + gadgeteerPosition.getValueAsDouble());

        /* Our feedback type matches the same type as the raw encoder getter, so position should be the same */
        System.out.println("Position " + regularPosition.getValueAsDouble());

        assertTrue(nearlyEqual(regularPosition.getValueAsDouble(), gadgeteerPosition.getValueAsDouble()));
    }

    private boolean nearlyEqual(double a, double b) {
        if (a > b + ALLOWABLE_DELTA) {
            return false;
        } else if (a < b - ALLOWABLE_DELTA) {
            return false;
        } else {
            return true;
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
}