package overcharged.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import overcharged.components.DuckDetector;
import overcharged.components.DuckPositions;
import overcharged.components.RotationAxis;
import overcharged.linear.components.RobotTankMecanumLinear;
import overcharged.linear.components.TankDriveLinear;
import overcharged.linear.util.SelectLinear;
import overcharged.linear.util.WaitLinear;
import overcharged.test.EasyOpenCVExample;

@Disabled
@Autonomous(name="mblueauto", preselectTeleOp = "mecanum teleop")
public class mblueauto extends LinearOpMode {

    private RobotTankMecanumLinear robot;
    private TankDriveLinear drive;
    SelectLinear sl = new SelectLinear(this);
    long currentTime;
    double startencoder;
    double level2;
    double level3;

    boolean switchBroken = false;

    double threelimit;
    double bottomlimit;
    boolean warehouse = true;
    DuckDetector detector;
    OpenCvWebcam webcam;
    slidesThread raiseSlidesThread;
    private DuckPositions duckPositions = DuckPositions.C;
    EasyOpenCVExample.RingDeterminationPipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            robot = new RobotTankMecanumLinear(this);
            drive = robot.getTankDriveLinear();
            WaitLinear lp = new WaitLinear(this);
            initCamera();
            startencoder = robot.getSlidePosition();
            level2 = startencoder+290;
            level3 = startencoder+600;

            telemetry.addData("Warehouse or Storage", warehouse ? "Warehouse" : "Storage");
            telemetry.update();
            warehouse = sl.selectWarehouse();

            this.detector = new DuckDetector();
            this.detector.useDefaults();
            if(warehouse==true){
                this.detector.setRange(0.14, 0.38, 0.62, 0.77);
                //(0.22, 0.46, 0.71, 0.77)
            } else{
                this.detector.setRange(0.26, 0.55, 0.82, 0.74);
            }
            webcam.setPipeline(detector);

            try {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            } catch(Exception e) {
                try {
                    this.detector = new DuckDetector();
                    this.detector.useDefaults();
                    if(warehouse==true){
                        this.detector.setRange(0.23, 0.47, 0.72, 0.78);
                    } else{
                        this.detector.setRange(0.28, 0.55, 0.81, 0.78);
                    }
                    webcam.setPipeline(detector);
                    webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                } catch (Exception f) {
                    telemetry.addLine("Error");
                    telemetry.update();
                    duckPositions = DuckPositions.C;
                }
            }

            while (!isStopRequested() && robot.gyroSensor.isCalibrating()) {
                telemetry.addData("Gyro Status", "Calibrating");
                telemetry.update();
                sleep(50);
                idle();
            }
            telemetry.addData("Gyro Status", "Calibrated");
            telemetry.update();

            long time1 = System.currentTimeMillis();
            currentTime = System.currentTimeMillis();
            while (currentTime - time1 < DuckDetector.DETECTION_WAIT_TIME) {
                duckPositions = detector.getDuckPositions();
                currentTime = System.currentTimeMillis();
            }

            detector.reset();
            telemetry.addData("Duck Position", duckPositions);
            telemetry.update();

            if (isStopRequested()) {
                return;
            }

            waitForStart();
            drive.resetAngle();
            if (opModeIsActive()) {
                time1 = System.currentTimeMillis();
                currentTime = System.currentTimeMillis();
                while (currentTime - time1 < DuckDetector.DETECTION_WAIT_TIME) {
                    duckPositions = detector.getDuckPositions();
                    currentTime = System.currentTimeMillis();
                }

                detector.reset();
                telemetry.addData("Duck Position", duckPositions);
                telemetry.update();

                webcam.stopStreaming();
                webcam.closeCameraDevice();

                checkSwitchBroken();
                raiseSlidesThread = new slidesThread(true, lp,this, robot, duckPositions, startencoder, level2, level3);

                if (warehouse==true){
                    warehouse(lp);
                } else{
                    storage(lp);
                }
            }

        } finally {
            // shut down
            if (robot != null) {
                robot.close();
            }
        }
    }

    /**
     * Camera Initialization Function
     */
    private void initCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new EasyOpenCVExample.RingDeterminationPipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDevice();
    }

    public void warehouse(WaitLinear lp) throws InterruptedException {
        robot.armMid();
        robot.cupLocked();
        drive.moveToEncoderInchUsingPID(3, 0.5f, 3000, true);
        drive.turnUsingPID(-28, 0.3f, RotationAxis.CENTER);
        raiseSlidesThread.run();

        if(duckPositions==DuckPositions.A) {
            drive.moveToEncoderInchUsingPID(23.5f, 0.3f, 3000, true);
        } else{
            drive.moveToEncoderInchUsingPID(21f, 0.5f, 3000, true);
        }
        //slideUp(lp);
        slideDown(lp);
        if(duckPositions==DuckPositions.A) {
            drive.moveToEncoderInchUsingPID(-28, 0.3f, 3000, true);
        } else{
            drive.moveToEncoderInchUsingPID(-26f, 0.5f, 3000, true);
        }
        drive.turnUsingPID(-39, 0.3f, RotationAxis.CENTER);
        drive.moveToEncoderInchUsingPID(-35, 0.7f, 3000, true);

        /*float distance=0;
        while(robot.isCollected()==false){
            drive.moveToEncoderInchUsingPID(-5, 0.3f, 3000, true);
            distance+=5;
        }
        robot.cupLocked();
        robot.outtake();
        lp.waitMillis(500);*/
    }

    /*public void warehouse(WaitLinear lp) throws InterruptedException {
        drive.moveToEncoderInchUsingPID(20, 0.7f, 3000, true);

        drive.moveWithPID(3, 0.5);
        drive.turnUsingPID(-27, 0.3f, RotationAxis.CENTER);
        drive.moveWithPID(23, 0.5);
        slideUp(lp);
        slideDown(lp);
        drive.moveWithPID(-30, 0.5);
        drive.turnUsingPID(-45, 0.3f, RotationAxis.CENTER);
        drive.moveWithPID(-55, 0.5);
    }*/

    public void storage(WaitLinear lp) throws InterruptedException {
        robot.armMid();
        robot.cupLocked();
        drive.moveToEncoderInchUsingPID(3, 0.5f, 3000, true);
        drive.turnUsingPID(19, 0.3f, RotationAxis.CENTER);
        raiseSlidesThread.run();

        drive.moveToEncoderInchUsingPID(20, 0.3f, 3000, true);
        //slideUp(lp);
        slideDown(lp);
        //drive.moveToEncoderInchUsingPID(-5, 0.3f, 3000, true);
        drive.turnUsingPID(34.5f, 0.3f, RotationAxis.CENTER);
        drive.moveToEncoderInchUsingPID(-40.5f, 0.3f, 3000, true);
        duck(lp);
        /*drive.moveToEncoderInchUsingPID(-5, 0.3f, 3000, true);
        drive.turnUsingPID(-20, 0.3f, RotationAxis.CENTER);
        drive.moveToEncoderInchUsingPID(-15, 0.7f, 3000, true);
        drive.turnUsingPID(-5, 0.3f, RotationAxis.CENTER);*/
        drive.turnUsingPID(-59, 0.3f, RotationAxis.CENTER);
        drive.moveToEncoderInchUsingPID(19, 0.62f, 3000, true);
    }

    public void parking(WaitLinear lp) throws InterruptedException {
        drive.moveToEncoderInchUsingPID(-15, 0.3f, 3000, true);
        drive.turnUsingPID(90, 0.3f, RotationAxis.CENTER);
        drive.moveToEncoderInchUsingPID(-30, 0.3f, 3000, true);
    }

    private void checkSwitchBroken() {
        if (((robot.getSlidePosition() - startencoder) > 200) && robot.isSlideSwitchPressed()) {
            switchBroken = true;
            robot.ledRedBlink();
        } else {
            switchBroken = false;
            robot.ledRedOn(false);
        }
    }

private boolean slideReachedBottom() {
        if (switchBroken) return robot.getSlidePosition() <= startencoder;
        return robot.isSlideSwitchPressed();
    }

    public void slideUp(WaitLinear lp) throws InterruptedException {
        //robot.cupLocked();
        double level;
        if(duckPositions==DuckPositions.A){
            level = startencoder+3;
        } else if(duckPositions==DuckPositions.B){
            level = level2;
        } else {
            level = level3;
        }
        while (robot.getSlidePosition() < level) {
            double distance = level - robot.getSlidePosition();
            //robot.slideOn(0.35);
            double calcPower = Math.signum(distance)*(Math.abs(distance)/level*3);
            robot.slideOn(Math.abs(calcPower) > 0.35 ? calcPower : Math.signum(calcPower)*0.35);
        }
        robot.slideOff();
        //lp.waitMillis(600);
        if(duckPositions==DuckPositions.A) {
            robot.armOutShared();
        } else{
            robot.armOut();
        }
        lp.waitMillis(700);
        robot.cupDump();
        lp.waitMillis(400);
        robot.cupOpen();
        lp.waitMillis(1000);
        robot.armMid();
        lp.waitMillis(300);
    }

    public void slideDown(WaitLinear lp) throws InterruptedException {
        while (!slideReachedBottom()) {
            robot.slideDown();
        }
        robot.slideOff();
        robot.cupOpen();
        robot.armDown();
    }

    public void duck(WaitLinear lp) throws InterruptedException {
        drive.moveToEncoderInchUsingPID(-2, 0.3f, 3000, true);
        robot.duckOn(0.35);
        lp.waitMillis(5000);
        robot.duckOff();
    }
}
