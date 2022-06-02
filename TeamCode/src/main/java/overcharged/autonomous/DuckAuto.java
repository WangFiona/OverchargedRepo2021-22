package overcharged.autonomous;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Vector;

import overcharged.components.DuckDetector;
import overcharged.components.DuckPositions;
import overcharged.components.RobotMecanum;
import overcharged.components.SensorDistance;
import overcharged.drive.AggressiveMecanumDrive;
import overcharged.drive.DriveConstants;
import overcharged.drive.SampleMecanumDrive;
import overcharged.linear.util.SelectLinear;
import overcharged.linear.util.WaitLinear;
import overcharged.opmode.mSlidesThread;
import overcharged.test.EasyOpenCVExample;
import overcharged.trajectorysequence.TrajectorySequence;

@Autonomous(name="Duck Auto")
public class DuckAuto extends LinearOpMode {

    RobotMecanum robot;
    SelectLinear sl = new SelectLinear(this);
    long currentTime;
    long startTime;
    double startencoder;
    double level2;
    double level3;
    boolean switchBroken = false;
    int cycleNumber = 0;
    double startDelaySecs;
    boolean blue = true;
    SensorDistance wallSensor;
    double hubX=-41;//-36;
    double duckX=-4;
    double duckY=13;
    double angle=86;

    DuckDetector detector;
    OpenCvWebcam webcam;
    private DuckPositions duckPositions = DuckPositions.C;
    EasyOpenCVExample.RingDeterminationPipeline pipeline;

    AggressiveMecanumDrive drive;
    TrajectorySequence goToCarousel, goToHub, goToDuck, intakeDuck, park;
    Pose2d start = new Pose2d();


    @Override
    public void runOpMode() throws InterruptedException {
        try {
            robot = new RobotMecanum(this, true, true);
            //robot.initFrontDistSensor(hardwareMap);
            //drive = new SampleMecanumDrive(hardwareMap);
            drive = new AggressiveMecanumDrive(hardwareMap);
            WaitLinear lp = new WaitLinear(this);
            initCamera();

            robot.armMid();
            robot.cupLocked();
            startencoder = robot.slides.getCurrentPosition();
            level2 = startencoder + 490;
            level3 = startencoder + 750;


            blue = sl.selectAlliance();

            startDelaySecs = sl.adjust("Delay Start", 20);
            startDelaySecs = startDelaySecs * 1000;

            final int INTAKE_SPEED = 15;

            //telemetry.addData("front distance:", robot.getDistanceF());

            /**
             * Create RoadRunner trajectories ahead of time.
             * goToCarousel starts at the initial position and moves to the carousel to deliver the duck.
             * From the carousel, goToHub goes to the hub and delivers the preload from the side
             * From the hub, goToDuck and intakeDuck goes back to the carousel, attempts to sweep the ground to pick up the duck, and delivers the duck to the hub
             * From the hub, park parks the robot for parking points.
             */
            goToCarousel = drive.trajectorySequenceBuilder(start)
                    .setReversed(true)
                    .lineTo(new Vector2d( duckX-4, 0))
                    .turn((blue ? 1 : -1) * (Math.toRadians(angle)))
                    .lineTo(new Vector2d(-2, (blue ? 1 : -1) * (duckY-2)))
                    .build();

            goToHub = drive.trajectorySequenceBuilder(new Pose2d(duckX, (blue ? 1 : -1) * (duckY), (blue ? 1 : -1) * Math.PI/2))
                    .setReversed(true)
                    .lineTo(new Vector2d((blue ? hubX : hubX+2), (blue ? 1 : -1) * (duckY+1)))
                    .addSpatialMarker(new Vector2d((blue ? hubX : hubX+2), (blue ? 1 : -1) * (duckY+1)), () -> {
                        raiseSlidesThread(lp);
                        robot.intakeOff();
                    })
                    .lineTo(new Vector2d((blue ? hubX+2 : hubX+2),duckPositions==DuckPositions.C ? ((blue ? -8 : 5)) : ((blue ? -7 : 25))))
                    .addSpatialMarker(new Vector2d((blue ? hubX : hubX+2), duckPositions==DuckPositions.C ? ((blue ? 1 : -1) * (-6)) : ((blue ? -6 : 24))), () -> {
                        robot.cupDump();
                    })
                    .build();

            goToDuck = drive.trajectorySequenceBuilder(goToHub.end())
                    .setReversed(false)
                    .lineTo(new Vector2d( hubX, (blue ? 1 : -1) * (duckY)))
                    .lineTo(new Vector2d(duckX-3, (blue ? 1 : -1) * (duckY)))
                    .turn(Math.toRadians(0))
                    .build();

            intakeDuck = drive.trajectorySequenceBuilder(new Pose2d(duckX-3, (blue ? 1 : -1) * (duckY), 0))
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(INTAKE_SPEED, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .strafeTo(new Vector2d(duckX-1,(blue ? 1 :-1)* (-6)))
                    .lineTo(new Vector2d(duckX+2,(blue ? 1 :-1)* (-6)))
                    .strafeTo(new Vector2d(duckX+2, (blue ? 1 : -1) * (duckY+2)))
                    .build();

            park = drive.trajectorySequenceBuilder(goToHub.end())
                    .setReversed(false)
                    .lineTo(new Vector2d( (blue ? hubX : hubX+4), (blue ? 1 : -1) * (duckY+5)))
                    .lineTo(new Vector2d((blue ? -26 : -29), (blue ? 1 : -1) * (duckY+5)))
                    .build();

            /**
             * Initialize the camera's team shipping element detector with settings.
             * These settings are tuned for the robot's position in the Duck side's initial position.
             */
            this.detector = new DuckDetector();
            this.detector.useDefaults();
            if (blue) {
                this.detector.setRange(0.01, 0.22, 0.45, 0.48);
                //this.detector.setRange(0.07, 0.28, 0.51, 0.53);
            } else {
                this.detector.setRange(0.27, 0.51, 0.77, 0.48);
            }
            webcam.setPipeline(detector);

            try {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            } catch (Exception e) {
                try {
                    this.detector = new DuckDetector();
                    this.detector.useDefaults();
                    if (blue) {
                        this.detector.setRange(0.01, 0.22, 0.45, 0.48);
                        //this.detector.setRange(0.07, 0.28, 0.51, 0.53);
                    } else {
                        this.detector.setRange(0.27, 0.51, 0.77, 0.48);
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

            /*long time1 = System.currentTimeMillis();
            currentTime = System.currentTimeMillis();
            while (currentTime - time1 < DuckDetector.DETECTION_WAIT_TIME) {
                duckPositions = detector.getDuckPositions();
                currentTime = System.currentTimeMillis();
            }*/

            detector.reset();
            telemetry.addData("Duck Position", duckPositions);
            telemetry.update();

            if (isStopRequested()) {
                return;
            }
            robot.ledGreenOn(true);


            waitForStart();

            robot.ledGreenOn(false);
            if (opModeIsActive()) {
                detectionWaitingStart = System.currentTimeMillis();
                while (System.currentTimeMillis() - detectionWaitingStart < DuckDetector.DETECTION_WAIT_TIME - 700) {
                    duckPositions = detector.getDuckPositions();
                }

                /*time1 = System.currentTimeMillis();
                currentTime = System.currentTimeMillis();
                while (currentTime - time1 < DuckDetector.DETECTION_WAIT_TIME) {
                    duckPositions = detector.getDuckPositions();
                    currentTime = System.currentTimeMillis();
                }*/

                detector.reset();
                telemetry.addData("Duck Position", duckPositions);
                telemetry.update();
                webcam.stopStreaming();
                webcam.closeCameraDevice();

                checkSwitchBroken();

                Duck(lp);
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
     * Main function for Duck side autonomous.
     * Delivers duck from carousel, delivers preload, intakes duck, delivers duck, and parks.
     */
    public void Duck(WaitLinear lp) throws InterruptedException {
        wallSensor = blue ? robot.sensorDistanceL : robot.sensorDistanceR;
        startTime = System.currentTimeMillis();
        robot.cupLocked();
        drive.setPoseEstimate(start);

        drive.followTrajectorySequence(goToCarousel);
        duck(lp);

        //Correct the angle if needed
        if(Math.abs(Math.abs(drive.getPoseEstimate().getHeading()-((blue ? Math.PI/2 : Math.PI*3/2)))) > Math.toRadians(2)){
            double currangle = (blue ? Math.PI/2 : Math.PI*3/2)-drive.getPoseEstimate().getHeading();
            drive.turn(currangle);
        }

        relocalizeXY();

        drive.followTrajectorySequence(goToHub);
        lowerSlidesThread(lp);

        //robot.intake.intake.setPower(0.75);
        robot.intakeOn();

        drive.followTrajectorySequence(goToDuck);

        //drive.turn(Math.toRadians(-angle));
        if(!robot.isCollected()){
            drive.followTrajectorySequenceAsync(intakeDuck);
            while(drive.isBusy() && !robot.cup.isDuckCollected() && opModeIsActive()){
                drive.update();
            }
            drive.setMotorPowers(0, 0, 0, 0);
        }


        robot.cupDuckLocked();
        lp.waitMillis(500);
        robot.intakeOff();
        armMidSlow(lp);


        /*if(!robot.cup.isDuckCollected()){
            drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineTo(new Vector2d(-27, (blue ? 1 : -1) * (duckY+6)))
                    .build()
            );
            stop();
        }*/

        duckY=duckY+2;
        drive.followTrajectorySequence(goToHub);
        lowerSlidesThread(lp);

        drive.followTrajectorySequence(park);
    }

    /**
     * Move the arm up slowly. If moved using the default speed, the duck can fling out.
     */
    private void armMidSlow(WaitLinear lp) throws InterruptedException {
        double mid = robot.arm.getMid();
        int nstep = 10;
        double step = mid/nstep;
        for (int i = 0; i < nstep; i++) {
            robot.arm.setPosition(i*step);
            lp.waitMillis(100);
        }
        robot.armMid();
    }
    private void checkSwitchBroken() {
        if (((robot.slides.getCurrentPosition() - startencoder) > 200) && robot.slides.isSlideSwitchPressed()) {
            switchBroken = true;
            robot.ledRedBlink();
        } else {
            switchBroken = false;
            robot.ledRedOn(false);
        }
    }

    private boolean slideReachedBottom() {
        if (switchBroken) return robot.slides.getCurrentPosition() <= startencoder;
        return robot.slides.isSlideSwitchPressed();
    }

    public void slideDown(WaitLinear lp) throws InterruptedException {
        while (!slideReachedBottom() && opModeIsActive()) {
            robot.slides.moveToBottom();
        }
        robot.cupOpen();
        robot.armDown();
    }

    /**
     * Asynchronously start raising slides. If in preload, deliver to based on team shipping element position.
     */
    public void raiseSlidesThread(WaitLinear lp) {
        Runnable raiseSlidesThread = new mSlidesThread(true, lp, this, robot, duckPositions, startencoder, level2, level3, cycleNumber);
        Thread thread = new Thread(raiseSlidesThread);
        thread.start();
        cycleNumber++;
    }

    /**
     * Asynchronously start raising slides
     */
    public void lowerSlidesThread(WaitLinear lp) {
        Runnable lowerSlidesThread = new mSlidesThread(false, lp, this, robot, duckPositions, startencoder, level2, level3, cycleNumber);
        Thread thread = new Thread(lowerSlidesThread);
        thread.start();
    }

    /**
     * Once at the carousel, spin the carousel to deliver the duck.
     */
    public void duck(WaitLinear lp) throws InterruptedException {
        long duckTime = System.currentTimeMillis();
        //telemetry.addData("front distance:", robot.getSonar());
        telemetry.update();
        while(robot.getSonar()>9 && System.currentTimeMillis()-duckTime<1000){
            //telemetry.addData("front distance:", robot.getSonar());
            telemetry.update();
            drive.setMotorPowers(0.2,0.2,0.2,0.2);
        }
        drive.setMotorPowers(0,0,0,0);
        duckTime = System.currentTimeMillis();
        while(System.currentTimeMillis()-duckTime<2300){
            robot.duckOn((blue ? 1 : -1) * (0.3));
        }
        robot.duckOn((blue ? 1 : -1) * (0.33));
        lp.waitMillis(2500);
        robot.duckOff();
    }

    /**
     * Relocalize x and y position using distance sensors.
     */
    public void relocalizeXY(){
        if(blue){
            double distance = wallSensor.getDistance();
            if(distance > 0 && distance < 10){
                drive.setPoseEstimate(
                        new Pose2d(
                                4 - distance,
                                drive.getPoseEstimate().getY(),
                                drive.getPoseEstimate().getHeading()
                        )
                );
            }
        }
        else{
            double distance = wallSensor.getDistance();
            if(distance > 0 && distance < 10){
                telemetry.addData("dist", distance);
                telemetry.update();
                drive.setPoseEstimate(
                        new Pose2d(
                                4 - distance,
                                drive.getPoseEstimate().getY(),
                                drive.getPoseEstimate().getHeading()
                        )
                );
            }

        }

        //double distToWall = robot.getDistanceF();
        float sonarDistance = (float) (((robot.sonar.getVoltage() * 6f * 1024f / 2.7f) - 300f) / 25.4f);
        double distToWall = (float) (0.9829f * sonarDistance - 0.5991);

        double y = (blue? 1: -1) * (23 - distToWall);

        if(distToWall > 5 && distToWall < 40){
            drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), y, drive.getPoseEstimate().getHeading()));
        }
        else{
            telemetry.addData("front range sensor relocalization was invalid", y);
        }
    }

}

