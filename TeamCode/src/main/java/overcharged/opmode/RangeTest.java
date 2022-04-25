package overcharged.opmode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.Arrays;
@Disabled
@TeleOp
public class RangeTest extends OpMode {
    Rev2mDistanceSensor rangeR;
    Rev2mDistanceSensor rangeL;
    //Rev2mDistanceSensor rangeF;

    ArrayList<Double> dists = new ArrayList<>();

    long start = -1;


    @Override
    public void init() {
        rangeR = hardwareMap.get(Rev2mDistanceSensor.class, "rangeR");
        rangeL = hardwareMap.get(Rev2mDistanceSensor.class, "rangeL");
        //rangeF = hardwareMap.get(Rev2mDistanceSensor.class, "rangeF");
    }



    @Override
    public void loop() {
        if(start == -1) start = System.currentTimeMillis();

        telemetry.addData("right range sensor", "%.2f mm, %.2f cm, %.2f in",
                rangeR.getDistance(DistanceUnit.MM),
                rangeR.getDistance(DistanceUnit.CM),
                rangeR.getDistance(DistanceUnit.INCH)
        );

        /*telemetry.addData("front range sensor", "%.2f mm, %.2f cm, %.2f in",
                rangeF.getDistance(DistanceUnit.MM),
                rangeF.getDistance(DistanceUnit.CM),
                rangeF.getDistance(DistanceUnit.INCH)
        );*/

        telemetry.addData("left range sensor", "%.2f mm, %.2f cm, %.2f in",
                rangeL.getDistance(DistanceUnit.MM),
                rangeL.getDistance(DistanceUnit.CM),
                rangeL.getDistance(DistanceUnit.INCH)
        );

        dists.add(rangeR.getDistance(DistanceUnit.INCH));

        if(System.currentTimeMillis() - start > 10000){

            double sum = 0;
            double min = 100000;
            double max = 0;
            for(Double d: dists)
            {
                sum += d;
                if(d >= max) max = d;
                if(d <= min) min = d;
            }

            double average = sum / dists.size();

            telemetry.addData("average", average);
            telemetry.addData("max", max);
            telemetry.addData("min", min);
            telemetry.update();


        }

        telemetry.update();

    }
}
