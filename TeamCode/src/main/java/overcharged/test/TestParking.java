package overcharged.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import overcharged.components.OcServo;

/**
 * Created by Parthiv Nair on 10/13/2018.
 * You can set the initial position using this program. This is useful especially when replacing servos
 */
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TestParking", group = "Tester")
public class TestParking extends OpMode {
    private OcServo servo;
    private ElapsedTime runtime = new ElapsedTime();

    int stopPosition = 127;
    int releasePosition = 210;
    @Override
    public void init() {
        hardwareMap.logDevices();
        //Initialize Servo
        try {
            servo = new OcServo(hardwareMap, "servo", stopPosition /255f);
            telemetry.addData("Status", "Servo initialized at " + stopPosition);
        } catch (Exception e) {
            telemetry.addData("Status", "missing: servo at port 0 " + e.getMessage());
        }
        telemetry.update();
    }

    int pos = stopPosition;
    String poss = "Stop ";
    public void loop() {
        long timestamp = System.currentTimeMillis();
        telemetry.addData("Status", "Running: " + runtime.toString());

        if (gamepad1.dpad_up) {
            poss= "Release ";
            pos = releasePosition;
            servo.setPosition(releasePosition /255f);
        } else {
            poss= "Stop ";
            pos = stopPosition;
            servo.setPosition(stopPosition /255f);
        }
        telemetry.addData("Position", poss + pos);
        telemetry.update();
    }
}