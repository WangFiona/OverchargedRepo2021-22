package overcharged.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

import overcharged.components.Button;
import overcharged.components.OcLed;
import overcharged.components.OcServo;
import overcharged.components.OcSwitch;
import overcharged.components.RobotMecanum;
import overcharged.components.Slide;
import overcharged.components.TurnType;
import overcharged.config.RobotConstants;
import overcharged.drive.SampleMecanumDrive;
import overcharged.linear.components.Robot6WheelLinear;
import overcharged.linear.components.TankDriveLinear;
import overcharged.test.MotorTestInfo;
import overcharged.test.ServoTestInfo;

/**
 * Overcharged Team #12599 Tester
 * This tester program has 16 separate tests.
 */
@Config
@TeleOp(name="Tester", group="Test")
public class
Tester
        extends LinearOpMode {

    ///Overcharged Autonomous Robot class
    private RobotMecanum robot;
    ///Overcharged Autonomous Tank Drive class
    //SampleMecanumDrive drive;
    //OcSwitch slideSwitch;
    //private TankDriveLinear drive;

    private ServoTestInfo[] servoTestInfos;
     /**
     * Counter of servos in servo test
     */
    private int servoTestCounter = 0;

    private final List<OcServo> servos = new ArrayList<>();

    /**
     * Counter for servos for servo calibrate test
     */
    private int servoCalibrateCounter = 0;

    private final static int MIN_SERVO_TICK = 1;
    private final static DecimalFormat integerFormatter = new DecimalFormat("######");
    private final static DecimalFormat decimalFormatter = new DecimalFormat("###.##");

    /**
     * Enumeration for the different tests
     */
    public enum ETest {
        NONE,
        SWITCH,
        ENCODER,
        MOTOR,
        DRIVE,
        SERVO_CALIBRATE,
        SERVO,
        GYRO,
        LED,
        DUCK,
        ;

        private static int numberTests = 0;

        /**
         * Get the test according to number
         * @param ordinal the test number
         * @return the test according to the number given
         */
        public static ETest getTest(int ordinal)
        {
            for (ETest e : values()) {
                if (e.ordinal() == ordinal) {
                    return e;
                }
            }

            return NONE;
        }

        /**
         * Get the number of tests
         * @return the number of tests
         */
        public static int getNumberTests() {
            if (numberTests == 0) {
                for (ETest ignored : values()) {
                    numberTests++;
                }
            }
            return numberTests;
        }
    }

    /**
     * Function for running tests
     */
    @Override
    public void runOpMode() {
        //Initialization
        robot = new RobotMecanum(this, true, false);
        //drive = robot.getTankDriveLinear();

        //drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        OcServo cup = new OcServo(robot.cup.cup,
                "cup",
                (float)robot.cup.open);
        servos.add(cup);
        OcServo cap = new OcServo(robot.cap.cap,
                "cap",
                (float)robot.cap.in);
        servos.add(cap);
        servoTestInfos = new ServoTestInfo[]{
            // cup
            new ServoTestInfo(
                    cup,
                    (float)robot.cup.dump,
                    (float)robot.cup.lockedBall,
                    (float)robot.cup.lockedBlock,
                    (float)robot.cup.lockedDuck),
            // cap
            new ServoTestInfo(
                    cap,
                    (float)(robot.cap.in - 0.05))
        };

        int testCounter = 0;
        ///Set current test to NONE
        ETest currentTest = ETest.NONE;

        telemetry.addData("Waiting", "Tester");
        telemetry.update();
        ///Waiting for start to be pressed
        waitForStart();

        while (opModeIsActive()) {
            long timeStamp = System.currentTimeMillis();

            ///Choosing the desired test
            if(gamepad1.right_trigger > 0.9 && Button.BTN_NEXT.canPress(timeStamp)) {
                testCounter++;
                if(testCounter >= ETest.getNumberTests()){
                    testCounter = 0;
                }
                currentTest = ETest.getTest(testCounter);
            } else if(gamepad1.left_trigger > 0.9 && Button.BTN_PREV.canPress(timeStamp)) {
                testCounter--;
                if(testCounter < 0){
                    testCounter = ETest.getNumberTests() - 1;
                }
                currentTest = ETest.getTest(testCounter);
            }

            telemetry.addData("Test", currentTest);
            telemetry.addData("Select", "Next:RightTrigger Prev:LeftTrigger");
            telemetry.addData("Run", "Start");

            ///Loop tests
            if (gamepad1.start && Button.BTN_START.canPress(timeStamp)) {
                switch(currentTest) {
                    case ENCODER:
                        encoderTest();
                        break;
                    case MOTOR:
                        motorTest();
                        break;
                    case DRIVE:
                        driveTest();
                        break;
                    case SERVO_CALIBRATE:
                        servoCalibrate(servos);
                        break;
                    case SERVO:
                        servoTest(servoTestInfos);
                        break;
                    case GYRO:
                        gyroTest();
                        break;
                    case SWITCH:
                        switchTest();
                        break;
                    case LED:
                        ledTest();
                        break;
                    case DUCK:
                        duckTest();
                        break;
                    case NONE:
                    default:
                        break;
                }
            }

            telemetry.update();
            idle();
        }
    }

    /**
     * Test for encoder values of all DC motors in the robot
     */
    private void encoderTest () {
        //Set all motors to FLOAT behavior while unpowered
        robot.drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.slides.slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.slides.slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        while (opModeIsActive()) {
            long timeStamp = System.currentTimeMillis();

            ///Reset encoders on all motors
            if (gamepad1.start && Button.BTN_START.canPress(timeStamp)) {

                robot.drive.resetPosition();
                robot.slides.resetSlidePosition();
                idle();
            }
            else if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp)) {
                break;
            }

            if (robot.slides.isSlideSwitchPressed()) {
                robot.slides.resetSlidePosition();
            }
            telemetry.addData("Test", "Encoders");
            telemetry.addData("Front",
                    "Left:" + integerFormatter.format(robot.driveLeftBack.getCurrentPosition()) +
                            " Right:" + integerFormatter.format(robot.driveRightBack.getCurrentPosition()));
            telemetry.addData("Back",
                    "Left:" + integerFormatter.format(robot.driveLeftFront.getCurrentPosition()) +
                            " Right:" + integerFormatter.format(robot.driveRightFront.getCurrentPosition()));
            telemetry.addData("Slides:",
                    integerFormatter.format(robot.slides.getCurrentPosition()));
            telemetry.addData("Reset", "Start");
            telemetry.addData("Back", "LeftStick");

            telemetry.update();
            idle();
        }
        ///Set drive train motors to BRAKE behavior while unpowered
        robot.drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        idle();
    }

    /**
     * Test the motors
     */
    private void motorTest () {
        float[] powers = new float[]{0.25f, -0.25f};

        MotorTestInfo[] motorTestInfos = new MotorTestInfo[]{new MotorTestInfo(robot.driveLeftFront, "driveLF"),
                new MotorTestInfo(robot.driveLeftBack,"driveLB"),
                new MotorTestInfo(robot.driveRightFront,"driveRF"),
                new MotorTestInfo(robot.driveRightBack, "driveRB")
        };

        back:
        for (MotorTestInfo motorTestInfo : motorTestInfos) {
            for (float power : powers) {
                motorTestInfo.stop();

                long startTimestamp = System.currentTimeMillis();
                long timeStamp = startTimestamp;

                while (opModeIsActive() &&
                        timeStamp - startTimestamp < 5000) {
                    telemetry.addData("Test", "Motors");
                    telemetry.addData("Motor", motorTestInfo.motorName);
                    telemetry.addData("Power", power);
                    telemetry.addData("CurrentPosition", motorTestInfo.motor.getCurrentPosition());

                    motorTestInfo.motor.setPower(power);

                    telemetry.addData("Reset", "Start");
                    telemetry.addData("Back", "LeftStick");

                    if (gamepad1.start && Button.BTN_START.canPress(timeStamp)) {

                        robot.drive.resetPosition();
                        //robot.slides.resetSlidePosition();
                        idle();
                    }
                    else if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp)) {
                        break back;
                    }

                    telemetry.update();
                    idle();
                    timeStamp = System.currentTimeMillis();
                }
                motorTestInfo.stop();
            }
        }

        idle();
    }

    /**
     * Test the limit switches at the bottom and top of the slide system
     */
    private void switchTest () {
        robot.slides.slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.slides.slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        while (opModeIsActive()) {
            long timeStamp = System.currentTimeMillis();

            if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp)) {
                break;
            }

            telemetry.addData("Test", "Switches");
            telemetry.addData(robot.slides.switchSlideDown.toString(), Boolean.toString(robot.slides.switchSlideDown.isTouch()));
            /*for (OcSwitch s : robot.switchs) {
                telemetry.addData(s.toString(), Boolean.toString(s.isTouch()));
            }*/

            telemetry.addData("Back", "LeftStick");

            telemetry.update();
            idle();
        }
    }

    /**
     * Calibration of servos in robot
     * @param servoCalibrateList servos to be tested
     */
    private void servoCalibrate(List<OcServo> servoCalibrateList) {
        int posJoy1 = (int)(servoCalibrateList.get(servoCalibrateCounter).getPosition() * 255f);

        while (opModeIsActive()) {
            long timeStamp = System.currentTimeMillis();

            ///Choose a servo for calibration
            if(gamepad1.right_trigger > 0.9 && Button.BTN_NEXT.canPress(timeStamp)) {
                servoCalibrateCounter++;
                if(servoCalibrateCounter >= servoCalibrateList.size()){
                    servoCalibrateCounter = 0;
                }
                posJoy1 = (int)(servoCalibrateList.get(servoCalibrateCounter).getPosition() * 255f);
            } else if(gamepad1.left_trigger > 0.9 && Button.BTN_PREV.canPress(timeStamp)) {
                servoCalibrateCounter--;
                if(servoCalibrateCounter < 0){
                    servoCalibrateCounter = servoCalibrateList.size() - 1;
                }
                posJoy1 = (int)(servoCalibrateList.get(servoCalibrateCounter).getPosition() * 255f);
            }
            else if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp)) {
                return;
            }

            ///Change servo position for calibration
            if (gamepad1.x && Button.BTN_MINUS.canPress4Short(timeStamp)) {
                posJoy1 -= MIN_SERVO_TICK;
            } else if (gamepad1.b && Button.BTN_PLUS.canPress4Short(timeStamp)) {
                posJoy1 += MIN_SERVO_TICK;
            } else if (gamepad1.y && Button.BTN_MAX.canPress(timeStamp)) {
                posJoy1 = 255;
            } else if (gamepad1.a && Button.BTN_MIN.canPress(timeStamp)) {
                posJoy1 = 0;
            } else if (gamepad1.right_stick_button && Button.BTN_MID.canPress(timeStamp)) {
                posJoy1 = 128;
            }
            posJoy1 = Range.clip(posJoy1, 0, 255);
            servoCalibrateList.get(servoCalibrateCounter).setPosition(posJoy1/255f);

            telemetry.addData("Test", "ServoCalibrate");
            telemetry.addData("Adjust", "+:B -:X Max:Y Min:A Mid:RStick");
            telemetry.addData("Position", integerFormatter.format(posJoy1));
            telemetry.addData("Servo", servoCalibrateList.get(servoCalibrateCounter));
            telemetry.addData("Select", "Next: RightTrigger Prev: LeftTrigger");
            telemetry.addData("Back", "LeftStick");

            telemetry.update();
            idle();
        }
    }
    /**
     * Tests each individual servo
     * @param servoTestInfos the servos to be tested
     */
    private void servoTest(ServoTestInfo[] servoTestInfos) {
        while (opModeIsActive()) {
            long timeStamp = System.currentTimeMillis();

            if (gamepad1.right_trigger > 0.9 && Button.BTN_NEXT.canPress(timeStamp)) {
                servoTestCounter++;
                if (servoTestCounter >= servoTestInfos.length) {
                    servoTestCounter = 0;
                }
            }
            else if (gamepad1.left_trigger > 0.9 && Button.BTN_PREV.canPress(timeStamp)) {
                servoTestCounter--;
                if (servoTestCounter < 0) {
                    servoTestCounter = servoTestInfos.length - 1;
                }
            }
            else if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp)) {
                return;
            }
            else if (gamepad1.start && Button.BTN_START.canPress(timeStamp)) {
                servoTest(servoTestInfos[servoTestCounter]);
            }

            telemetry.addData("Test", "Servo");
            telemetry.addData("Select", "Next:RightTrigger Prev:LeftTrigger");
            telemetry.addData("Servo", servoTestInfos[servoTestCounter].servo);
            telemetry.addData("Run", "Start");
            telemetry.addData("Back", "LeftStick");

            telemetry.update();
            idle();
        }
    }

    /**
     * Tests each individual servo
     * @param servoTestInfo the servos to be tested
     */
    private void servoTest(ServoTestInfo servoTestInfo) {
        for (int i = 0; i <= servoTestInfo.positions.length; i++) {
            float currentPosition = servoTestInfo.servo.getPosition();
            float position;
            if (i < servoTestInfo.positions.length) {
                position = servoTestInfo.positions[i];
            }
            else {
                position = servoTestInfo.servo.getInitialPosition();
            }

            long startTimestamp = System.currentTimeMillis();
            long timeStamp = startTimestamp;
            long timeout = (long)(Math.abs(position - currentPosition) * 1000 * servoTestInfo.timeScale);

            while (opModeIsActive() &&
                    timeStamp - startTimestamp < timeout) {
                servoTestInfo.servo.setPosition(position);

                if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp)) {
                    return;
                }

                telemetry.addData("Test", "Servo");
                telemetry.addData("Position", integerFormatter.format(servoTestInfo.servo.getPosition()));
                telemetry.addData("Servo", servoTestInfo.servo);
                telemetry.addData("Back", "LeftStick");

                telemetry.update();
                idle();
                timeStamp = System.currentTimeMillis();
            }
        }
    }

    /**
     * Test the drive train, motors, and servos
     */
    private void driveTest () {
        float[][] powers = new float[][]{
                {0.25f, 0.25f},
                {-0.25f, -0.25f},
                {0.25f, -0.25f},
                {-0.25f, 0.25f},
        };

        back:
        for (float[] power : powers) {
            robot.drive.resetPosition();

            long startTimestamp = System.currentTimeMillis();
            long timeStamp = startTimestamp;

            while (opModeIsActive() &&
                    timeStamp - startTimestamp < 5000) {

                robot.drive.setPower(power[0], power[1]);

                telemetry.addData("Test", "Drive");
                telemetry.addData("Front",
                        "Left:" + integerFormatter.format(robot.driveLeftFront.getCurrentPosition()) +
                                " Right:" + integerFormatter.format(robot.driveRightFront.getCurrentPosition()));
                telemetry.addData("Back",
                        "Left:" + integerFormatter.format(robot.driveLeftBack.getCurrentPosition()) +
                                " Right:" + integerFormatter.format(robot.driveRightBack.getCurrentPosition()));
                telemetry.addData("Back", "LeftStick");

                if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp)) {
                    break back;
                }

                telemetry.update();
                idle();
                timeStamp = System.currentTimeMillis();
            }

            robot.drive.stop();
        }

        robot.drive.stop();
        robot.drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        idle();
    }

    /**
     * Test showing IMU headings
     */
    private void gyroTest() {
        robot.drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        float headingLImuBase = 0;
        float headingRImuBase = 0;

        while (opModeIsActive()) {
            long timeStamp = System.currentTimeMillis();

            Orientation anglesLImu = robot.gyroSensor.gyroL.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            float headingLImu = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(anglesLImu.angleUnit, anglesLImu.firstAngle));
            Orientation anglesRImu = robot.gyroSensor.gyroL.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            float headingRImu = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(anglesRImu.angleUnit, anglesRImu.firstAngle));

            if (gamepad1.start && Button.BTN_START.canPress(timeStamp)) {

                headingLImuBase = headingLImu;
                headingRImuBase = headingRImu;
                idle();
            }
            else if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp)) {
                break;
            }

            telemetry.addData("Test", "Gyro");
            telemetry.addData("gyroLeft",
                    decimalFormatter.format(headingLImu - headingLImuBase));
            telemetry.addData("gyroRight",
                    decimalFormatter.format(headingRImu - headingRImuBase));
            telemetry.addData("Reset", "Start");
            telemetry.addData("Back", "LeftStick");

            telemetry.update();
            idle();
        }
        robot.drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        idle();
    }

    private void ledTest() {
        telemetry.addData("Test", "LED");
        telemetry.update();

        for (OcLed led: robot.leds) {
            led.on();
            led.draw();
            sleep(800);
        }
        sleep(2000);
        for (OcLed led: robot.leds) {
            sleep(400);
            led.off();
            led.draw();
        }
        telemetry.addLine("All LED tested");
        telemetry.update();

        idle();
    }

    private void duckTest() {
        final int TIME = 3000;
        float power = 0.3f;
        back:
        for (int i = 0; i < 2; i++) {
            long startTime = System.currentTimeMillis();
            long timeStamp = startTime;

            while (opModeIsActive() && timeStamp - startTime < TIME) {
                robot.duckNoPID.duck.setPower(power);

                if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp)) {
                    break back;
                }

                telemetry.addData("Test", "Duck");
                telemetry.addData("Back", "LeftStick");
                telemetry.update();
                idle();
                timeStamp = System.currentTimeMillis();
            }
            power = -power;
        }

        // stop the carousel
        robot.duckNoPID.duck.setPower(0);
        idle();
    }
}