package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.drivetrains.MecanumLocalizer;
import com.pedropathing.localization.constants.OdometryConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    // --- Follower Constants ---
    // Mass in kg: weigh robot OR weigh yourself holding robot then subtract your weight
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(16.2);

    // --- Mecanum Drivetrain Constants ---
    // Motor names MUST match exactly what you defined in the Robot Configuration app.
    // Directions must match your hardware setup (check RobotHardware.java for confirmation).
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1.0)
            .leftFrontMotorName("FL")
            .leftRearMotorName("BL")
            .rightFrontMotorName("FR")
            .rightRearMotorName("BR")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);

    // --- Odometry / Localization Constants ---
    // This uses your drive motors as the localizer (no dead wheels).
    // If you add dead wheels, see the "Dead Wheel Localizer" section of the tutorial.
    public static OdometryConstants odometryConstants = new OdometryConstants()
            // PIDF coefficients for fusing motor encoder data into pose estimates
            // Default values are a starting point — tune them in Panels or FTC Dashboard.
            .odometryCoefficients(new PIDFCoefficients(0.2, 0, 0, 0))
            .parallelTickCoefficients(new PIDFCoefficients(0.15, 0, 0, 0))
            .perpendicularTickCoefficients(new PIDFCoefficients(0.15, 0, 0, 0))
            .headingTickCoefficients(new PIDFCoefficients(1.0, 0, 0, 0))
            // forwardAsX/strafeAsY: set to false if your robot's "forward" maps to
            // the y-axis on the field instead of the x-axis. Tune with LocalizationTest.
            .forwardAsX(false)
            .strafeAsY(false)
            // Ticks to inches: circumference / counts per revolution
            // GoBILDA 312 RPM: 60mm omni = 0.0597 m = 2.35 inches per revolution
            // REV 20.4:1 HD Hex 2 counts per rev → adjust to your actual wheels
            .parallelTickToInches(1.0)   // TODO: set to measured value (see Tuning Guide)
            .strafeTickToInches(1.0);    // TODO: set to measured value

    // --- Path Constraints ---
    // tValue: path completion percentage (0.995 = 99.5% before considering "done")
    // velocity: max speed in in/sec
    // translational: max lateral error in inches before path stalls
    // heading: max heading error in radians (0.009 rad ≈ 0.5°)
    // timeout: max ms to correct after reaching end before declaring done
    // deceleration: how aggressively to slow down (higher = sharper braking)
    // breakRatio: fraction of path to apply braking (1.0 = entire path)
    // breakMinVelocity: in/sec — speed below which braking is skipped
    public static PathConstraints pathConstraints = new PathConstraints(
            0.995, 100.0, 1.0, 0.009, 50_000, 1.25, 10, 1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .localizer(new MecanumLocalizer(hardwareMap, odometryConstants))
                .pathConstraints(pathConstraints)
                .build();
    }
}
