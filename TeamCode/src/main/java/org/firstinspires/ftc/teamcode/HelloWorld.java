package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Disabled
@Autonomous(name="Hello World", group="Test")
public class HelloWorld extends OpMode {

    @Override
    public void init() {
        telemetry.addData("Hello", "World");
    }

    @Override
    public void loop() {

    }
}

