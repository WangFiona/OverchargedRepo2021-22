package overcharged.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import overcharged.components.DuckPositions;
import overcharged.linear.components.Robot6WheelLinear;
import overcharged.linear.components.RobotTankMecanumLinear;
import overcharged.linear.util.WaitLinear;

import static overcharged.config.RobotConstants.TAG_A;

public class slidesThread implements Runnable {
    private RobotTankMecanumLinear robot;
    private WaitLinear lp;
    private LinearOpMode opMode;

    double startencoder;
    double level2;
    double level3;
    boolean raiseUp=true;
    boolean switchBroken = false;

    private DuckPositions duckPositions = DuckPositions.C;

    public slidesThread(boolean up, WaitLinear wl, LinearOpMode mode, RobotTankMecanumLinear r, DuckPositions d, double se, double two, double three) {
        // store parameter for later use
        raiseUp=up;
        lp = wl;
        opMode = mode;
        robot = r;
        duckPositions=d;
        startencoder=se;
        level2=two;
        level3=three;
    }

    public void run() {
        try {
            if (!opMode.isStopRequested()) {
                RobotLog.v(TAG_A, "Start raising slides");
                long startTime = System.currentTimeMillis();

                if(raiseUp==true){
                    slideUp(lp);
                } else{
                    slideDown(lp);
                }

                long TimeElapsed = System.currentTimeMillis() - startTime;
                RobotLog.ii(TAG_A, "Stop raising slides " + (System.currentTimeMillis() - startTime) + " milliseconds");
            }
        } catch (InterruptedException e) {
            RobotLog.ii(TAG_A, "Error: " + e.getStackTrace());
        }
    }

    public void slideUp(WaitLinear lp) throws InterruptedException {
        //robot.cupLocked();
        if(duckPositions == DuckPositions.C){ robot.armOut();}
        double level;
        if(duckPositions==DuckPositions.A){
            level = startencoder+15;
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
        if(duckPositions==DuckPositions.B) {
            robot.armOutShared();
        } else if(duckPositions==DuckPositions.A){
            robot.armOutAuto();
        }
        /*else{
            robot.armOut();
        }*/
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

    public void slideDown(WaitLinear lp) throws InterruptedException {
        while (!slideReachedBottom()) {
            robot.slideDown();
        }
        robot.slideOff();
        robot.cupOpen();
        robot.armDown();
    }
}