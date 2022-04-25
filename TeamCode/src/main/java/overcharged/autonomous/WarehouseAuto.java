package overcharged.autonomous;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

import overcharged.components.DuckDetector;
import overcharged.components.DuckPositions;
import overcharged.components.RobotMecanum;
import overcharged.components.SensorDistance;
import overcharged.drive.DriveConstants;
import overcharged.drive.SampleMecanumDrive;
import overcharged.linear.util.SelectLinear;
import overcharged.linear.util.WaitLinear;
import overcharged.opmode.mSlidesThread;
import overcharged.test.EasyOpenCVExample;
import overcharged.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name="Warehouse Auto")
public class WarehouseAuto extends LinearOpMode {

    RobotMecanum robot; // stores all peripherals on the robot (everything on the ctrl hub/exp hub except for movement)
    SelectLinear sl = new SelectLinear(this); // lets drivers select options before auto starts

    long startTime;
    double startencoder; // encoder position when the slides are down
    double level2;
    double level3;

    boolean switchBroken = false;
    int cycleNumber = 0; // used because we don't want to deliver based on detection after preload


    boolean blue = true; // trajectories must be mirrored on red
    double startDelayMillis = 0; // delay in the warehouse/init pos so the alliance can deliver preload
    double startDelaySecs = 0;

    DuckDetector detector;
    OpenCvWebcam webcam;

    private DuckPositions duckPositions = DuckPositions.C;
    EasyOpenCVExample.RingDeterminationPipeline pipeline;

    SampleMecanumDrive drive; // for RoadRunner and movement
    TrajectorySequence goToHub, goToHub_A, goToWarehouse, splineToHub; // move around using RoadRunner

    Pose2d start = new Pose2d();

    Vector2d hub, hub_A; // hub position
    Vector2d warehouse;

    LapTimer lapTimer = new LapTimer(telemetry); // Acts like a stopwatch and times laps whenever lap is called. Logs in telemetry

    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry telems;


    @Override
    public void runOpMode() throws InterruptedException {
        try {
            telems = new MultipleTelemetry(dashboard.getTelemetry(), telemetry);


            robot = new RobotMecanum(this, true, true);
            WaitLinear lp = new WaitLinear(this);
            initCamera(); // initializes camera and sets up pipeline for team shipping element detection

            // when we start, we want the outtake mechanism in a predictable position
            robot.armMid();
            robot.cupLocked();

            startencoder = robot.slides.getCurrentPosition(); // slide should be down here so we initialize this as "0"

            level2 = startencoder + 490; // for mid
            level3 = startencoder + 750; // for highest


            drive = new SampleMecanumDrive(hardwareMap);


            blue = sl.selectAlliance();

            startDelaySecs = sl.adjust("Delay Start", 20);
            startDelayMillis = startDelaySecs * 1000;


            hub = new Vector2d(-19, (blue ? 1 : -1) * 19); // where the hub is to dump
            hub_A = new Vector2d(-20, (blue ? 1 : -1) * 19); // where we should dump on the lowest level (this needs to be different because the slides don't move out on preload low)

            warehouse = new Vector2d(2, (blue ? 1 : -1) * -33); // where we should start intaking in the warehouse


            final int INTAKE_SPEED = 15; // speed when intaking, needs to be slower to prevent jamming
            final int FAST_SPEED = 60; // speed whenever we CAN go really fast, usually when hitting walls


            /**
             * Create RoadRunner trajectories ahead of time to save time once auto starts.
             * goToHub(_A) starts at the initialized position, strafes to the hub, and dumps the preload block. _A is necessary because preload needs a different dump position
             * splineToHub starts at the warehouse, and splines out of the hub and to the hub, making sure not to hit the barrier on the way out.
             * goToWarehouse starts at the hub, and splines into the warehouse, effectively the opposite of splineToHub
             */
            goToHub = drive.trajectorySequenceBuilder(start)
                    .lineTo(hub)
                    .addSpatialMarker(new Vector2d(hub.getX() + 2, hub.getY() - (2 * (blue ? 1 : -1))), () -> {
                        robot.cupDump();
                    })
                    .build();
            goToHub_A = drive.trajectorySequenceBuilder(start)
                    .lineTo(hub_A)
                    .addSpatialMarker(new Vector2d(hub.getX() + 2, hub.getY() - (2 * (blue ? 1 : -1))), () -> {
                        robot.cupDump();
                    })
                    .build();
            splineToHub = drive.trajectorySequenceBuilder(new Pose2d(warehouse.getX(), warehouse.getY(), (blue ? 1 : -1) * -Math.PI / 2))
                    .setReversed(true)
                    .lineTo(new Vector2d(warehouse.getX(), -23 * (blue ? 1 : -1)))

                    .addSpatialMarker(new Vector2d(0, (blue ? 1 : -1) * -27), () -> {
                        robot.outtake();// outtake when we're leaving the hub
                    })

                    .addSpatialMarker(new Vector2d(0, (blue ? 1 : -1) * -10), () -> {
                        // stop outtaking a little later and start raising slides
                        robot.intakeOff();
                        raiseSlidesThread(lp);
                    })
                    .splineTo(hub, Math.toRadians(180))
                    .build();
            goToWarehouse = drive.trajectorySequenceBuilder(splineToHub.end())
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(FAST_SPEED, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    // go to the warehouse as fast as possible
                    .setReversed(false)
                    .splineTo(new Vector2d(warehouse.getX(), (blue ? 1 : -1) * -28), ((blue ? 1 : -1) * (-Math.PI / 2)))

                    // slow down because we don't want to jam
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(INTAKE_SPEED * 2.25, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .addSpatialMarker(new Vector2d(warehouse.getX(), (blue ? 1 : -1) * -20), () -> {
                        robot.intakeOn();
                    })
                    // go to (30,-80) beacuse this is past the wall, meaning we'll do wall following to align ourselves
                    .lineTo(new Vector2d(30, -80 * (blue ? 1 : -1))) // keep going forward (slowly), this will get cancelled prematurely if the cup intakes
                    .build();


            /**
             * Initialize the webcam and Team Shipping Element OpenCV detector in 320x240px.
             * Use different filter positions for red/blue because the camera is not centerred on the robot, meaning the shipping element will be in a different position
             */
            this.detector = new DuckDetector(); // for detecting team shipping element before auto starts
            this.detector.useDefaults();
            if (blue) {
                this.detector.setRange(0.07, 0.28, 0.51, 0.53);
            } else {
                this.detector.setRange(0.33, 0.57, 0.83, 0.55);
            }
            webcam.setPipeline(detector);

            try {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            } catch (Exception e) {
                try {
                    this.detector = new DuckDetector();
                    this.detector.useDefaults();
                    if (blue) {
                        this.detector.setRange(0.07, 0.28, 0.51, 0.53);
                    } else {
                        this.detector.setRange(0.33, 0.57, 0.83, 0.55);
                    }
                    webcam.setPipeline(detector);
                    webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                } catch (Exception f) {
                    telemetry.addLine("Error");
                    telemetry.update();
                    duckPositions = DuckPositions.C;
                }
            }


            long detectionWaitingStart = System.currentTimeMillis();
            while (System.currentTimeMillis() - detectionWaitingStart < DuckDetector.DETECTION_WAIT_TIME + 500) {
                duckPositions = detector.getDuckPositions();
            }

            detector.reset();
            telemetry.addData("Duck Position", duckPositions);
            telemetry.update();

            if (isStopRequested()) {
                return;
            }
            robot.ledGreenOn(true);

            waitForStart();
            lapTimer.reset();


            robot.ledGreenOn(false);
            if (opModeIsActive()) {
                detectionWaitingStart = System.currentTimeMillis();
                while (System.currentTimeMillis() - detectionWaitingStart < DuckDetector.DETECTION_WAIT_TIME - 700) {
                    duckPositions = detector.getDuckPositions();
                }

                detector.reset();
                telemetry.addData("Duck Position", duckPositions);
                telemetry.update();

                webcam.stopStreaming();
                webcam.closeCameraDevice();

                checkSwitchBroken();

                lp.waitMillis(startDelayMillis);

                blueWarehouse(lp); // deliver preload, and keep doing cycles as long as we have time
            }

        } catch (Exception e) {
            RobotLog.e("Overcharged: Autonomous Failed: ", e.getMessage());
        } finally {
            // shut down
            if (robot != null) {
                robot.close();
            }
        }
    }

    private void initCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new EasyOpenCVExample.RingDeterminationPipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDevice();
    }

    /**
     * Main function for autonomous. Once run, the robot will start by delivering preload.
     * After, the robot will keep "cycling" (intaking from the warehouse and delivering) until there's less than 3.5 of the 30 seconds left.
     * 3.5 seconds means we make sure we get the points for parking.
     * Additionally, not attempting another cycle prevents the robot from stopping with the slides up.
     * If TeleOp starts with the slides up, the limit switch won't hit, messing up the automation heights.
     */
    public void blueWarehouse(WaitLinear lp) throws InterruptedException {
        SensorDistance wallSensor = blue ? robot.sensorDistanceL : robot.sensorDistanceR; // use a diff side sensor based on which side we are on

        startTime = System.currentTimeMillis();

        robot.cupLocked();

        drive.setPoseEstimate(start); // resets position

        raiseSlidesThread(lp); // asyncly start raising the slides

        if (duckPositions == DuckPositions.A) { // goes to the hub for the first preload cycle
            drive.followTrajectorySequence(goToHub_A);
        } else {
            drive.followTrajectorySequence(goToHub);
        }

        lp.waitMillis(200); // wait 200 ms before lowering because we don't want to lower as we're delivering

        lowerSlidesThread(lp); // starts lowering slide bc we're going to leave the hub

        lapTimer.lap("start -> hub"); // lap this so we can see it in telem later

        robot.outtake(); // if there's anything from a previous cycle, we can start outtaking just to prevent a potential jam

        drive.followTrajectorySequenceAsync(goToWarehouse);

        warehouseIntaking(lp); // controls the goToWarehouse trajectory
        double collectedTime = System.currentTimeMillis();

        lapTimer.lap("hub -> collected (first cycle)");


        while (System.currentTimeMillis() - startTime < 26500 && opModeIsActive()) { // keep running this code until we have <3.5 seconds left (~1 cycle)
            relocalizeUsingDistance(wallSensor, lp); // relocalize to maintain accuracy between cycles


            drive.setMotorPowers(0, 0, 0, 0);


            drive.followTrajectorySequenceAsync(splineToHub); // start following the hub but do it async so we can control it
            long extraIntakeTime = System.currentTimeMillis();
            robot.intakeOn();
            while (System.currentTimeMillis() - extraIntakeTime < 300 && opModeIsActive()) {
                drive.update();
            }
            robot.outtake();
            // intake for an extra 300ms, then follow the rest of the trajectory

            while (drive.isBusy() && opModeIsActive()) {
                drive.update();
            }

            robot.cupDump(); // dump when we get to the hub
            lp.waitMillis(250); // let robot finish dumping


            lapTimer.lap("warehouse -> hub");


            lowerSlidesThread(lp);

            robot.outtake();

            drive.followTrajectorySequenceAsync(goToWarehouse);
            warehouseIntaking(lp);

            lapTimer.lap("hub -> warehouse");
        }

        robot.intakeOff();
        drive.setMotorPowers(0, 0, 0, 0);

        lapTimer.log(); // log to telemetry. Turn off timer on DS to easily view this.
        while (!isStopRequested()) {
            idle();
        }

    }

    /**
     * Check if the slide's limit switch is broken once before autonomous starts.
     * If the encoder position is past a certain position (200 ticks) and the switch is still not pressed, the switch is broken.
     */
    private void checkSwitchBroken() { // checks if the limit switch is broken
        if (((robot.slides.getCurrentPosition() - startencoder) > 200) && robot.slides.isSlideSwitchPressed()) {
            switchBroken = true;
            robot.ledRedBlink();
        } else {
            switchBroken = false;
            robot.ledRedOn(false);
        }
    }

    public void raiseSlidesThread(WaitLinear lp) { // asynchronously start raising the slides
        Runnable raiseSlidesThread = new mSlidesThread(true, lp, this, robot, duckPositions, startencoder, level2, level3, cycleNumber);
        Thread thread = new Thread(raiseSlidesThread);
        thread.start();
        cycleNumber++;
    }

    public void lowerSlidesThread(WaitLinear lp) { // asynchronously start lowering the slides
        Runnable lowerSlidesThread = new mSlidesThread(false, lp, this, robot, duckPositions, startencoder, level2, level3, cycleNumber);
        Thread thread = new Thread(lowerSlidesThread);
        thread.start();
    }

    void warehouseIntaking(WaitLinear lp) throws InterruptedException { // controls Roadrunner inside the warehouse
        long startMoveTime = System.currentTimeMillis();
        // blindly move for 1s. This makes sure we've delivered the last element and the cup is empty
        while (System.currentTimeMillis() - startMoveTime < 1000 && opModeIsActive()) {
            drive.update();
        }

        // keep moving until 4 seconds passes OR the cup is full, whichever happens first
        while (!robot.isCollected() && (System.currentTimeMillis() - startMoveTime < 4000) && opModeIsActive()) {
            drive.update();
        }

        // if we still haven't collected, try outtaking and then intaking to get the element. This usually fixes jams

        if (!robot.isCollected()) {
            robot.outtake();
            long recycle = System.currentTimeMillis();
            while (System.currentTimeMillis() - recycle < 800 && opModeIsActive()) {
                if (robot.isCollected()) {
                    break;
                }
                idle();
            }

            robot.intakeOn();
            recycle = System.currentTimeMillis();
            while (System.currentTimeMillis() - recycle < 800 && opModeIsActive()) {
                if (robot.isCollected()) {
                    break;
                }
                idle();
            }
        }


        drive.setMotorPowers(0, 0, 0, 0); // we're about to do relocalization. Make sure the robot is stopped
        robot.intakeOff();

        lp.waitMillis(150);
        robot.intakeOn();


        robot.cupLocked();
    }

    // For doing time splits during auto
    class Lap {
        String name;
        double time;
    }


    /**
     * The LapTimer class allows us to do time-splits live during autonomous.
     * Throughout many iterations of the autonomous program, time was the biggest constraint.
     * By making time-splits live, we can analyze how long each part of the autonomous took.
     */
    class
    LapTimer {
        double start;
        double last;
        Telemetry t;
        ArrayList<Lap> laps;

        public LapTimer(Telemetry t) {
            this.start = System.currentTimeMillis();
            this.last = start;
            this.t = t;
            laps = new ArrayList<>();

        }

        public void lap(String name) {
            Lap l = new Lap();
            l.name = name;
            l.time = System.currentTimeMillis() - this.last;
            laps.add(l);
            this.last = System.currentTimeMillis();
        }

        public void log() {
            for (Lap l : laps) {
                t.addData(l.name, l.time);
            }
            t.update();
        }

        public void reset() {
            this.start = System.currentTimeMillis();
        }
    }

    /**
     * Use the front and side sensor to relocalize the x and y position.
     * Initially, the wall sensor is polled up to 3 times with a 150ms timeout. If all 3 polls fail (0 > distance > 10), the robot uses predetermined values.
     * Second, the front sonar sensor value is read (voltage) and is tuned for accuracy.
     * Lastly, the x and y value are converted to RoadRunner coordinates and are set to override the current RoadRunner position
     */
    public void relocalizeUsingDistance(SensorDistance wallSensor, WaitLinear lp) throws InterruptedException{
        double distance = wallSensor.slowPoll(lp, 0, 10); // poll wall distance up to 3 times. if it still doesn't work, use a preset value


        if (distance == -1) {
            distance = blue ? 3.19 : 1.80; // this was manually found using RangeTest
            telemetry.addLine("wall sensor failed");
            telemetry.update();
        }

        drive.setMotorPowers(0, 0, 0, 0);
        /*
        double distToWall = robot.sensorDistanceF.slowPoll(lp, 1, 48);
        if(distToWall == -1){
            stop();
        }
        double y = (blue? 1: -1) * (-54.25 + distToWall);*/

        DistanceValue sonarDistance = getSonar();
        DistanceValue rev2MDistance = get2M(robot.sensorDistanceF, 0, 48);

        double usedDistance;

        // TODO is 12 good???
        if(sonarDistance.value < 12 || !rev2MDistance.valid){
            usedDistance = sonarDistance.value;
        }
        else{
            usedDistance = rev2MDistance.value;
        }

        double y = (blue ? 1 : -1) * (-65 + usedDistance);




        /*float sonarDistance = (float) (((robot.sonar.getVoltage() * 6f * 1024f / 2.7f) - 300f) / 25.4f);
        double distToWall = (float) (0.9829f * sonarDistance - 0.5991);
        // use predetermined sonar tuning values to find the distance to the wall.
        // this can't be preset like side wall and is important for relocalization so sonar (slower) is used.

        double y = (blue ? 1 : -1) * (-65 + distToWall);*/


        drive.setPoseEstimate(new Pose2d(4 - distance, y, drive.getPoseEstimate().getHeading())); // updated RoadRunner with front/back pos
    }



    public DistanceValue get2M(SensorDistance sensor, double min, double max){

        for(int i = 0; i < 10; i++){
            double value = robot.sensorDistanceF.getDistance();
            if(value > min && value < max){
                return new DistanceValue(value, true);
            }
        }
        return new DistanceValue(false);
    }

    public DistanceValue getSonar(){
        double distance = robot.getSonar();
        return new DistanceValue(distance, true);
    }




    public class DistanceValue{
        public double value;
        public boolean valid;
        public DistanceValue(double value, boolean valid){
            this.valid = valid;
            this.value = value;
        }

        public DistanceValue(boolean valid){
            this.valid = valid;
        }
    }
}
