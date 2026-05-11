package org.firstinspires.ftc.teamcode.samples;


import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class PedroTeleOpSample extends CommandOpMode {
    Follower follower;
    TelemetryData telemetryData = new TelemetryData(telemetry);

    @Override
    public void initialize() {
        follower = Constants.createFollower(hardwareMap);
        super.reset();

        follower.startTeleopDrive();
    }

    @Override
    public void preRun() {
        follower.update();

        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading", follower.getPose().getHeading());
        telemetryData.update();
    }

    @Override
    public void run() {
        super.run();
        follower.update();
    }
}