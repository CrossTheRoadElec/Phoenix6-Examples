package frc.robot.sysid;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Voltage;

public class SysIdSwerveTranslation implements SwerveRequest {
    public final MutableMeasure<Voltage> VoltsToApply = mutable(Volts.of(0));

    private VoltageOut m_voltRequest = new VoltageOut(0);

    public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
        for (int i = 0; i < modulesToApply.length; ++i) {
            modulesToApply[i].applyCharacterization(Rotation2d.fromDegrees(0), m_voltRequest.withOutput(VoltsToApply.in(Volts)));
        }
        return StatusCode.OK;
    }

    public SysIdSwerveTranslation withVolts(Measure<Voltage> Volts) {
        VoltsToApply.mut_replace(Volts);
        return this;
    }
}
