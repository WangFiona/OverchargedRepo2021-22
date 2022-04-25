package overcharged.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import overcharged.components.Button;
import overcharged.components.Robot6Wheel;

@Disabled
@Config
@TeleOp(name="sixwheelteleop")
public class sixwheelteleop extends OpMode {

    Robot6Wheel robot;

    // Intake States
    boolean intakeOn = false;
    boolean intakeOut = false;

    // Duck State
    boolean duckOn = false;

    // Slide State
    boolean slideUp = false;
    boolean slideDown = false;
    boolean slide = false; //true=up false=down
    double startencoder;
    //int startencoder;
    int flag = 0;
    double limit = 100;

    // Cup State
    boolean cupLocked = false;

    double powerMult = 1;
    boolean isSlow = false;

    public static double p = 20;
    public static double i = 0;
    public static double d = 0;
    public static double f = 0;

    public static double pwr = 0.51925;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot6Wheel(this, false);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }

    @Override
    public void loop() {
        if(robot.slideencoder==null){
            telemetry.addLine("no limit switch");
            telemetry.update();
            return;
        }
        if (flag==1) {
            startencoder = robot.slideencoder.getPosition();
            flag++;
        }
        long timestamp = System.currentTimeMillis();

        robot.drive.setTank(-gamepad1.left_stick_y * powerMult, -gamepad1.right_stick_y * powerMult);
        //robot.slide.setPIDFCoefficients(p,i,d,f);

        if(gamepad1.right_bumper && Button.BTN_SLOW_MODE.canPress(timestamp)){
            if(!isSlow){
                powerMult = 0.4;
                isSlow = true;
            } else {
                powerMult = 1;
                isSlow = false;
            }
        }

        // Intake Controls
        if(gamepad1.right_trigger > 0.9 && Button.BTN_AUTO.canPress(timestamp)){
            robot.cupOpen();
            if(!intakeOn){
                robot.intakeOn();
                intakeOn = true;
            } else {
                robot.intakeOff();
                intakeOn = false;
            }
            intakeOut = false;
        }
        if(gamepad1.left_trigger > 0.9 && Button.BTN_AUTOMATION.canPress(timestamp)){
            robot.cupOpen();
            if(!intakeOut){
                robot.intakeOut();
                intakeOut = true;
            } else {
                robot.intakeOff();
                intakeOut = false;
            }
            intakeOn = false;
        }

        //Cup controls
        if(gamepad1.left_bumper && Button.BTN_BACK.canPress(timestamp)){
            //if slide is up, toggle between locked and dump
            if(slide==true){
                if(!cupLocked) {
                    robot.cupLocked();
                    cupLocked = true;
                } else {
                    robot.cupDump();
                    cupLocked = false;
                }
            } else{ //if slide is down, toggle between locked and open
                if(!cupLocked) {
                    robot.cupLocked();
                    cupLocked = true;
                } else {
                    robot.cupOpen();
                    cupLocked = false;
                }
            }
        }

        //Duck controls
        if(gamepad2.b && Button.BTN_BACK.canPress(timestamp)){
            //robot.duckOn(0.7);
            if(!duckOn){
                robot.duckOn(0.52);
                duckOn = true;
            } else {
                robot.duckOff();
                duckOn = false;
            }
        } /*else{
            robot.duck.duck.setPower(0);
            //robot.duckOff();
        }*/

        if(gamepad2.x && Button.BTN_BACK.canPress(timestamp)){
            //robot.duckOn(-0.7);
            if(!duckOn){
                robot.duckOn(-0.52);
                duckOn = true;
            } else {
                robot.duckOff();
                duckOn = false;
            }
        } /*else{
            robot.duck.duck.setPower(0);
        }*/

        /*int onelimit = startencoder+100;
        int twolimit = startencoder+300;
        int threelimit = startencoder+600;*/

        double onelimit = startencoder+500;
        double twolimit = startencoder+1000;
        double threelimit = startencoder+1600;

        double bottomlimit = startencoder+50;
        //Slide controls
        if(gamepad2.dpad_up && Button.BTN_DISABLE.canPress(timestamp)){
            if(!slideUp && robot.slideencoder.getPosition()<onelimit){
                robot.cupLocked();
                cupLocked=true;
                robot.slideUp();
                slideUp = true;
                limit = onelimit;
                slide=true;
            } else{
                robot.slideOff();
                slideUp=false;
            }
        }

        if(gamepad2.dpad_right && Button.BTN_DISABLE.canPress(timestamp)){
            if(!slideUp && robot.slideencoder.getPosition()<twolimit){
                robot.cupLocked();
                cupLocked=true;
                robot.slideUp();
                slideUp = true;
                limit = twolimit;
                slide=true;
            } else{
                robot.slideOff();
                slideUp=false;
            }
        }

        if(gamepad2.dpad_left && Button.BTN_DISABLE.canPress(timestamp)){
            if(!slideUp && robot.slideencoder.getPosition()<threelimit){
                robot.cupLocked();
                cupLocked=true;
                robot.slideUp();
                slideUp = true;
                limit = threelimit;
                slide=true;
            } else{
                robot.slideOff();
                slideUp=false;
            }
        }

        if(slideUp && robot.slideencoder.getPosition()>=limit){
            robot.slideOff();
            slideUp = false;
        }

        if(gamepad2.dpad_down && Button.BTN_DISABLE.canPress(timestamp)){
            if(!slideDown && robot.slideencoder.getPosition()>bottomlimit) {
                //if(robot.isPressed()==false){
                robot.slideDown();
                slideDown = true;
                robot.cupOpen();
                cupLocked =false;
                slide=false;
            } else{
                robot.slideOff();
                slideDown=false;
            }
        }

        if(slideDown && robot.slideencoder.getPosition()<=bottomlimit){
            //if(robot.isPressed()==true){
            robot.slideOff();
            slideDown = false;
        }

        /*if (robot.isPressed()==false) {
            telemetry.addData("Digital Touch", "Is Not Pressed");
        } else {
            telemetry.addData("Digital Touch", "Is Pressed");
            //robot.slideOff();
            //slideDown = false;
        }*/

        telemetry.addData("Cup Open", cupLocked);
        telemetry.addData("Slow Mode", isSlow);
        //telemetry.addData("Slide Drift left", Math.abs(robot.slide.getDrift(robot.slide.left)));
        telemetry.update();
    }
}