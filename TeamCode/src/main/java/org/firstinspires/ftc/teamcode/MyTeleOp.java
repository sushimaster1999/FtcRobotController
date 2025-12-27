package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name="Hello World TeleOp", group="Test")
public class MyTeleOp extends OpMode {
    @Override
    public void init() {
        telemetry.addData("This is", "The TeleOp");
    }

    @Override
    public void loop() {

    }
    // ... methods for the TeleOp class
}
