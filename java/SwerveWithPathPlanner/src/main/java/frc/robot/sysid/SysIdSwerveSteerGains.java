package frc.robot.sysid;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Voltage;

public class SysIdSwerveSteerGains implements SwerveRequest {
    public final MutableMeasure<Voltage> VoltsToApply = mutable(Volts.of(0));

    private VoltageOut m_voltRequest = new VoltageOut(0);

    public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
        for (int i = 0; i < modulesToApply.length; ++i) {
            modulesToApply[i].getSteerMotor().setControl(m_voltRequest.withOutput(VoltsToApply.in(Volts)));
            modulesToApply[i].getDriveMotor().setControl(m_voltRequest.withOutput(0));
        }
        return StatusCode.OK;
    }

    public SysIdSwerveSteerGains withVolts(Measure<Voltage> Volts) {
        VoltsToApply.mut_replace(Volts);
        return this;
    }
}
