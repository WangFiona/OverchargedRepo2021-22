package overcharged.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import overcharged.components.RotationAxis;
import overcharged.linear.components.Robot6WheelLinear;
import overcharged.linear.components.RobotTankMecanumLinear;
import overcharged.linear.components.TankDriveLinear;
import overcharged.linear.util.WaitLinear;

import static overcharged.config.RobotConstants.TAG_A;

/*
 * Overcharged Team #12599 Autonomous
 */

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "curveturntest", group = "Test")
public class curveturntest extends LinearOpMode {
    ///Overcharged Autonomous Robot class
    private RobotTankMecanumLinear robot;
    ///Overcharged Swerve Drive class
    private TankDriveLinear drive;

    float power = .80f;
    double r = 0;
    double a = 0;
    double traveled=0;

    /**
     * Autonomous opMode, entry point into the program
     */
    @Override
    public void runOpMode() throws InterruptedException {
        try {
            // init
            robot = new RobotTankMecanumLinear(this);
            RobotLog.ii(TAG_A, "RobotTankMecanumLinear initialized");
            drive = robot.getTankDriveLinear();
            RobotLog.ii(TAG_A, "TankDriveLinear initialized");
            drive.yesReset(false);
            drive.resetAngle();
            run();
        } finally {
            // shut down
            if (robot != null) {
                robot.close();
            }
        }
    }

    /**
     * Autonomous run function
     * This is the main function that performs all the actions at once
     * @throws InterruptedException
     */
    public void run() throws InterruptedException {
        WaitLinear lp = new WaitLinear(this);
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && robot.gyroSensor.isCalibrating())
        {
            sleep(50);
            idle();
        }
        telemetry.addData("Waiting", "Autonomous");
        telemetry.addData("DistanceR: ", robot.getDistanceR());
        telemetry.addData("DistanceL: ", robot.getDistanceL());
        telemetry.addData("DistanceF: ", robot.getDistanceF());
        telemetry.update();
        waitForStart();

        //The time when the autonomous run started i.e. the start/stop button was pressed
        long startTime = System.currentTimeMillis();

        //return immediately if stop has been pressed instead of play
        if (isStopRequested()) {
            return;
        }

        ///If the user presses stop, waitForStart() will return, only run if the start button is pressed (not stop).
        if (opModeIsActive()) {
            //drive.curveTurnUsingPID(50, 0.5, 10,true,true);

            drive.curveTurnUsingPID(-90, 0.5, 10,false,true);
            //telemetry.addData("DistanceR: ", robot.getDistanceR());
            //telemetry.addData("DistanceL: ", robot.getDistanceL());
            //telemetry.addData("DistanceF: ", robot.getDistanceF());
            telemetry.update();
        }
    }
}