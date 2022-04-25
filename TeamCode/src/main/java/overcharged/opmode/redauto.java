package overcharged.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import overcharged.components.DuckDetector;
import overcharged.components.DuckPositions;
import overcharged.components.RingDetector;
import overcharged.components.RingPosition;
import overcharged.components.RotationAxis;
import overcharged.linear.components.Robot6WheelLinear;
import overcharged.linear.components.TankDriveLinear;
import overcharged.linear.util.SelectLinear;
import overcharged.linear.util.WaitLinear;
import overcharged.test.EasyOpenCVExample;

import static overcharged.config.RobotConstants.TAG_A;

@Disabled
@Autonomous(name="redauto", preselectTeleOp = "mecanum teleop")
public class redauto extends LinearOpMode {

    private Robot6WheelLinear robot;
    private TankDriveLinear drive;
    SelectLinear sl = new SelectLinear(this);
    private boolean isRed = true;
    long currentTime;
    double startencoder;
    double onelimit;
    double twolimit;
    double threelimit;
    double bottomlimit;
    boolean warehouse = true;
    DuckDetector detector;
    OpenCvWebcam webcam;
    private DuckPositions duckPositions = DuckPositions.C;
    EasyOpenCVExample.RingDeterminationPipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            robot = new Robot6WheelLinear(this);
            drive = robot.getTankDriveLinear();
            WaitLinear lp = new WaitLinear(this);
            initCamera();
            startencoder = robot.slideencoder.getPosition();
            onelimit = startencoder+300;
            twolimit = startencoder+800;
            threelimit = startencoder+1600;
            bottomlimit = startencoder+50;

            telemetry.addData("Warehouse or Storage", warehouse ? "Warehouse" : "Storage");
            telemetry.update();
            warehouse = sl.selectWarehouse();

            this.detector = new DuckDetector();
            this.detector.useDefaults();
            if(warehouse==true){
                this.detector.setRange(0.33, 0.58, 0.83, 0.72);
            } else{
                this.detector.setRange(0.27, 0.54, 0.81, 0.72);
            }
            webcam.setPipeline(detector);

            try {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            } catch(Exception e) {
                try {
                    this.detector = new DuckDetector();
                    this.detector.useDefaults();
                    if(warehouse==true){
                        this.detector.setRange(0.33, 0.58, 0.83, 0.72);
                    } else{
                        this.detector.setRange(0.33, 0.58, 0.83, 0.72);
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
        drive.moveToEncoderInchUsingPID(-3, 0.5f, 3000, true);
        drive.turnUsingPID(23, 0.3f, RotationAxis.CENTER);
        drive.moveToEncoderInchUsingPID(-23, 0.5f, 3000, true);
        slideUp(lp);
        slideDown(lp);
        drive.moveToEncoderInchUsingPID(8, 0.3f, 3000, true);
        drive.turnUsingPID(42, 0.3f, RotationAxis.CENTER);
        drive.moveToEncoderInchUsingPID(55, 0.7f, 3000, true);
    }

    public void storage(WaitLinear lp) throws InterruptedException {
        drive.moveToEncoderInchUsingPID(-3, 0.5f, 3000, true);
        drive.turnUsingPID(-20, 0.3f, RotationAxis.CENTER);
        /*if (duckPositions == DuckPositions.C){
            drive.moveToEncoderInchUsingPID(-18, 0.3f, 3000, true);
            drive.turnUsingPID(50, 0.3f, RotationAxis.CENTER);
            drive.turnUsingPID(-50, 0.3f, RotationAxis.CENTER);
            drive.moveToEncoderInchUsingPID(-4, 0.3f, 3000, true);
        } else{*/
        drive.moveToEncoderInchUsingPID(-22, 0.3f, 3000, true);
        //}
        slideUp(lp);
        slideDown(lp);
        drive.moveToEncoderInchUsingPID(4, 0.3f, 3000, true);
        drive.turnUsingPID(-25, 0.3f, RotationAxis.CENTER);
        drive.moveToEncoderInchUsingPID(34, 0.3f, 3000, true);
        duck(lp);
        drive.moveToEncoderInchUsingPID(-9, 0.3f, 3000, true);
        drive.turnUsingPID(152, 0.3f, RotationAxis.RIGHT);
        drive.moveToEncoderInchUsingPID(-9, 0.3f, 3000, true);

    }

    public void parking(WaitLinear lp) throws InterruptedException {
        drive.moveToEncoderInchUsingPID(-15, 0.3f, 3000, true);
        drive.turnUsingPID(-90, 0.3f, RotationAxis.CENTER);
        drive.moveToEncoderInchUsingPID(-30, 0.3f, 3000, true);

        /*this.detector = new RingDetector(!isRed);
        this.detector.useDefaults();
        this.detector.setRedRange(0.1, 115, 28);
        this.detector.setBlueRange(0.35, 33, 120);
        webcam.setPipeline(detector);

        try {
            webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        } catch(Exception e) {
            try {
                this.detector = new RingDetector(!isRed);
                this.detector.useDefaults();
                this.detector.setRedRange(0.1, 115, 28);
                this.detector.setBlueRange(0.15, 33, 120);
                webcam.setPipeline(detector);
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            } catch (Exception f) {
                ringPosition = RingPosition.C;
            }
        }

        long time1 = System.currentTimeMillis();
        currentTime = System.currentTimeMillis();
        while (opModeIsActive() && currentTime - time1 < RingDetector.DETECTION_WAIT_TIME) {
            ringPosition = detector.getRingPosition();
            currentTime = System.currentTimeMillis();
        }

        detector.reset();
        telemetry.addData("Ring Position=", ringPosition);
        telemetry.update();

        webcam.stopStreaming();
        webcam.closeCameraDevice();*/
    }

    public void slideUp(WaitLinear lp) throws InterruptedException {
        double limit;
        if(duckPositions==DuckPositions.A){
            limit = onelimit;
        } else if(duckPositions==DuckPositions.B){
            limit = twolimit;
        } else{
            limit = threelimit;
        }
        while (robot.slideencoder.getPosition() < limit) {
            robot.slideUp();
        }
        robot.slideOff();
        robot.cupDump();
        lp.waitMillis(500);
    }

    public void slideDown(WaitLinear lp) throws InterruptedException {
        while (robot.slideencoder.getPosition() > bottomlimit) {
            robot.slideDown();
        }
        robot.slideBrake();
        robot.cupOpen();
    }

    public void duck(WaitLinear lp) throws InterruptedException {
        drive.moveToEncoderInchUsingPID(2, 0.3f, 3000, true);
        robot.duckOn(0.5);
        lp.waitMillis(2500);
        robot.duckOff();
    }
}
