package overcharged.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.TimeUnit;

import overcharged.components.Arm;
import overcharged.components.Button;
import overcharged.components.OcSwitch;
import overcharged.components.RobotMecanum;
import overcharged.components.Slide;
import overcharged.pid.Controller;

@Disabled
@TeleOp(name="ServoTesting")
public class ServoTesting extends OpMode {

    double trackwidth = 18.1; // 819.654713 ticks
    double TICK_TO_INCH = 45.2847908;

    double left = 0;
    double right = 0;

    double startLF = 0, startLB = 0, lf = 0, lb = 0;

    RobotMecanum robot;

    Controller pidRotate, pidDrive;

    boolean isSpline = false;
    boolean firstAngle = false;
    double startAngle;

    double slidePos = 0;

    @Override
    public void init() {
        robot = new RobotMecanum(this, false,false);
        robot.cap.isCapOut = true;
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        long timestamp = System.currentTimeMillis();

        /*float x1 = gamepad1.left_stick_x;
        float y1 = gamepad1.left_stick_y;
        float x2 = gamepad1.right_stick_x;
        float y2 = gamepad1.right_stick_y;

        robot.drive.setArcade(x1,y1,x2);

        lf = robot.driveLeftFront.getCurrentPosition() - startLF;
        lb = robot.driveLeftBack.getCurrentPosition() - startLB;

        left = (lf + lb)/2;

        if(gamepad1.left_bumper && Button.BTN_LENGTHWISE.canPress(timestamp)){
            splineWhile(850, 31, 0.9, true);
        }
        if(gamepad1.right_bumper && Button.BTN_LENGTHWISE.canPress(timestamp)){
            splineWhile(-800, 60, 0.4, false);
        }
        if(isSpline){
            spline(1600,-34,0.7);
        }*/

        if(gamepad1.right_stick_y != 0){
            robot.setCapPos(gamepad1.right_stick_y);
        }

//        if(gamepad1.left_stick_y < 0 && robot.getSlidePosition() < 615){
//            robot.slideOn(-gamepad1.left_stick_y);
//        } else if(gamepad1.left_stick_y > 0 && !robot.isSlideSwitchPressed()) {
//            robot.slideOn(-0.8*gamepad1.left_stick_y);
//        } else if(!robot.isSlideSwitchPressed()) {
//            robot.slide.keep();
//        } else {
//            robot.slideOff();
//        }

        telemetry.addData("cap position", robot.slide.cap.getPosition());
        telemetry.update();
    }

    public void spline(double r, double theta, double power){
        if(isSpline){
            double leftPower;
            double rightPower;
            double c = r/(r+819.654713);
            double desiredAngle;
            double currentAngle = robot.gyroSensor.getFirstAngle();
            if(!firstAngle){
                startAngle = robot.gyroSensor.getFirstAngle();
                firstAngle = true;
            }

            if(r < 0){
                leftPower = power*c;
                rightPower = power;
                desiredAngle = startAngle+theta;
            } else {
                leftPower = power;
                rightPower = power*c;
                desiredAngle = startAngle-theta;
            }

            if(!inRange(desiredAngle, currentAngle, 2)) {
                /*double constant = Math.signum(desiredAngle-currentAngle)*Math.abs(desiredAngle-currentAngle)/Math.abs(desiredAngle-startAngle);
                setLeftPower(Math.abs(leftPower*constant) > 0.2 ? leftPower*constant : Math.signum(leftPower*constant)*0.2);
                setRightPower(Math.abs(rightPower*constant) > 0.2 ? rightPower*constant : Math.signum(rightPower*constant)*0.2);*/
                setLeftPower(leftPower);
                setRightPower(rightPower);
            } else {
                isSpline = false;
                firstAngle = false;
                robot.drive.stop();
            }
        }
    }

    public void splineWhile(double r, double theta, double power, boolean isReverse){
        double leftPower;
        double rightPower;
        double c = Math.abs(r)/(Math.abs(r)+819.654713);
        double desiredAngle;
        double currentAngle = robot.gyroSensor.getFirstAngle();
        if(!firstAngle){
            startAngle = robot.gyroSensor.getFirstAngle();
            firstAngle = true;
        }

        if(r < 0){
            leftPower = power*c;
            rightPower = power;
        } else {
            leftPower = power;
            rightPower = power*c;
        }

        desiredAngle = startAngle+theta;

        while(!inRange(desiredAngle, currentAngle, 0.5)) {
            currentAngle = robot.gyroSensor.getFirstAngle();
            double constantCalc = 1.2*(desiredAngle-currentAngle)/Math.abs(desiredAngle-startAngle);
            double constant = Math.abs(constantCalc) > 0.2 ? constantCalc : Math.signum(constantCalc)*0.2;
            setLeftPower(isReverse ? -1*leftPower*constant : leftPower*constant);
            setRightPower(isReverse ? -1*rightPower*constant : rightPower*constant);
        }
        firstAngle = false;
    }

    public double difference(double x, double y){
        return Math.abs(x-y);
    }

    public double tickToInch(double tick){
        return tick/TICK_TO_INCH;
    }

    public void setLeftPower(double power){
        robot.driveLeftFront.setPower(power);
        robot.driveLeftBack.setPower(power);
    }

    public void setRightPower(double power){
        robot.driveRightFront.setPower(power);
        robot.driveRightBack.setPower(power);
    }

    public boolean inRange(double x, double y, double threshhold){
        if(Math.abs(x-y) < threshhold) return true;
        return false;
    }
}
