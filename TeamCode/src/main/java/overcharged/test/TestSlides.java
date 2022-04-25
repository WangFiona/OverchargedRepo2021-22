package overcharged.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.DecimalFormat;

import overcharged.components.Button;
import overcharged.components.Slides;
import overcharged.util.PIDCalculator;

/**
 * Created by Parthiv Nair on 12/10/2019.
 */

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TestSlides", group = "Test")
public class TestSlides extends OpMode {
    ///Declare the slides component
    private Slides slides;
    private final static DecimalFormat numberFormatter = new DecimalFormat("######");
    private ElapsedTime runtime = new ElapsedTime();
    ///Initialize the PIDCalculator
    private PIDCalculator pidController = new PIDCalculator(0.25, 0, 0);
    private boolean moveslidetobottom = false;
    @Override
    public void init() {
        hardwareMap.logDevices();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //Initialize Motors
        try {
            slides = new Slides(null, hardwareMap, null);
            telemetry.addData("Status", "Initialized");
        } catch (Exception e) {
            telemetry.addData("Status", "Error: " + e.getMessage());
        }
        telemetry.update();
    }

    @Override
    public void loop() {
        long timestamp = System.currentTimeMillis();
        telemetry.addData("Status", "Running: " + runtime.toString());
        telemetry.addData("Test", "Use gamepad2 dpad U/R/D and left joystick");
        telemetry.addData("Stop", "Back");
        float arm_y1 = -gamepad2.left_stick_y;
        //arm_y1 = Button.scaleInput(arm_y1);

        //Slide controls
        if (gamepad2.dpad_up && Button.BTN_SLIDE_UP.canPress(timestamp)) {
            //moveSlidesTo(600);
            moveslidetobottom = false;
            slides.moveToTop();
        } else if (gamepad2.dpad_down && Button.BTN_SLIDE_DOWN.canPress(timestamp)) {
            moveslidetobottom = true;
//            slides.moveToBottom();
        } else if (gamepad2.dpad_right && Button.BTN_UP.canPress(timestamp)) {
            moveslidetobottom = false;
            slides.moveToMid();
        } else if (arm_y1 != 0) {
            moveslidetobottom = false;
            slides.move(arm_y1);
        } else if (!moveslidetobottom){
            slides.keep();
        }
        if (moveslidetobottom) {
            slides.moveToBottom();
        }
        telemetry.addData("Encoder Left", numberFormatter.format(slides.slideLeft.getCurrentPosition()));
        telemetry.addData("Encoder Right", numberFormatter.format(slides.slideRight.getCurrentPosition()));
        telemetry.addData("Current Left", numberFormatter.format(slides.currentPositionL));
        telemetry.addData("Current Right", numberFormatter.format(slides.currentPositionR));
        telemetry.addData("Run Mode", slides.slideLeft.getMode());
        telemetry.addData("Velocity Left", numberFormatter.format(slides.slideLeft.getVelocity()));
        telemetry.addData("Velocity Right", numberFormatter.format(slides.slideRight.getVelocity()));
        telemetry.addData("Stop", "Back");
        if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timestamp)) {
            telemetry.addData("Stop", "True");
            stop();
        }
        telemetry.update();
    }
}
