package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Pathing.PathController;
import org.firstinspires.ftc.teamcode.RobotHardware.Hardware;

@TeleOp
public class ExampleNewStuff extends LinearOpMode {

    // This runs as soon as init it pushed.
    @Override
    public void runOpMode() throws InterruptedException {
        hw = new Hardware(hardwareMap);
        hw.imuPos.reset();
        // wait for start to be pushed.
        waitForStart();

        PathController pathing = new PathController(hw, this, 0.5);

        pause();
        pathing.setTargetHeadingDeg(45);
        pathing.setTargetLocation(24,24);
        pathing.run();

        pause();
        pathing.setTargetHeadingDeg(-90);
        pathing.setTargetLocation(36,24);
        pathing.run();
    }

    private void pause() {
        while(!gamepad1.b)
            sleep(250);
    }

    Hardware hw;
    double power = 0.5;
}
