package first.robot;

import org.wpilib.smartdashboard.Field2d;
import org.wpilib.smartdashboard.Mechanism2d;
import org.wpilib.smartdashboard.MechanismLigament2d;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryTable;
import org.wpilib.util.Color;
import org.wpilib.util.Color8Bit;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

public class SwerveTelemetry {
    private final double MAX_SPEED;

    /* Table for swerve telemetry publishing */
    private final TelemetryTable swerveTelem = Telemetry.getTable("Swerve");

    /* Robot pose on a field */
    private final Field2d field = new Field2d();

    /* Mechanisms to represent the swerve module states */
    private final Mechanism2d[] moduleMechanisms = new Mechanism2d[] {
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
    };
    /* A direction and length changing ligament for speed representation */
    private final MechanismLigament2d[] moduleSpeeds = new MechanismLigament2d[] {
        moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
    };
    /* A direction changing and length constant ligament for module direction */
    private final MechanismLigament2d[] moduleDirections = new MechanismLigament2d[] {
        moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.WHITE))),
        moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.WHITE))),
        moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.WHITE))),
        moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.WHITE))),
    };

    /**
     * Constructs a swerve telemetry object.
     * 
     * @param maxSpeed Maximum speed of the robot in meters per second
     */
    public SwerveTelemetry(double maxSpeed) {
        MAX_SPEED = maxSpeed;
        /* optionally start the SignalLogger immediately */
        // SignalLogger.start();
    }

    /** Accepts the swerve drive state and telemeterizes it to SignalLogger and Telemetry. */
    public void telemeterize(SwerveDriveState state) {
        /* Write the swerve drive state to the log file */
        SignalLogger.writeStruct("DriveState", state.getStruct(), state, state.Timestamp);

        /* Also telemeterize the swerve drive state */
        swerveTelem.log("DriveState", state, state.getStruct());

        /* Telemeterize the pose to a Field2d */
        field.setRobotPose(state.Pose);
        swerveTelem.log("Pose", field);

        /* Telemeterize each module state to a Mechanism2d */
        for (int i = 0; i < moduleMechanisms.length; ++i) {
            moduleDirections[i].setAngle(state.ModuleVelocities[i].angle);
            moduleSpeeds[i].setAngle(state.ModuleVelocities[i].angle);
            moduleSpeeds[i].setLength(state.ModuleVelocities[i].velocity / (2 * MAX_SPEED));

            swerveTelem.log("Module " + i, moduleMechanisms[i]);
        }
    }
}
