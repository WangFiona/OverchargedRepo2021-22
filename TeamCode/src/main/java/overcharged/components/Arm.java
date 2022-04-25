package overcharged.components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Arm {
    public Servo left;
    public Servo right;

    double downR = 0.07;
    double downL = 1.0;

    double mid = 0.21;
    double out = 0.36;
    double outShared = 0.4;
    double outSharedReach = 0.5;
    double outSharedReachLess = 0.45;

    double outSharedAutoR = 0.643;
    double outSharedAutoL = 0.427;



    double autoOut = 0.373;
    double autoOutforR = 0.621;
    double autoOutforL = 0.449;

    double autoOutforA = 0.42;
    double autoOutforAR = 0.668;
    double autoOutforAL = 0.402;

    public boolean armOut;
    public boolean armDown;

    //double adjust = 0.0615;

    public static double adjust = 0.15772105;//0.5454;

    public Arm(HardwareMap hardwareMap, boolean isAutonomous){
        downR += adjust;
        downL -= adjust;

        right = hardwareMap.servo.get("armR");
        left = hardwareMap.servo.get("armL");
        if(isAutonomous){
            setMid();
            armDown = false;
        } else {
            setDown();
            armDown = true;
        }

        armOut = false;
    }

    public void setDown(){
        setPosition(0);
        armOut = false;
        armDown = true;
    }

    public void setMid(){
        setPosition(mid);
        armOut = false;
        armDown = false;
    }
    public double getMid(){
        return mid;
    }
    public void setOut(){
        setPosition(out);
        armOut = true;
        armDown = false;
    }

    public void setOutShared(){
        setPosition(outShared);
        armOut = true;
        armDown = false;
    }

    public void setOutSharedReach(){
        setPosition(outSharedReach);
        armOut = true;
        armDown = false;
    }

    public void setOutSharedReachLess(){
        setPosition(outSharedReachLess);
        armOut = true;
        armDown = false;
    }

    public void setAutoOut() {
        setOut();/*
        left.setPosition(autoOutforL);
        right.setPosition(autoOutforR);
        armOut = true;
        armDown = false;*/
    }

    public void setAutoShared(){
        setOutShared(); /*
        left.setPosition(outSharedAutoL);
        right.setPosition(outSharedAutoR);
        armOut = true;
        armDown = false;*/
    }

    public void setAutoOutforA() {
        setOutShared(); /*
        left.setPosition(autoOutforAL);
        right.setPosition(autoOutforAR);
        armOut = true;
        armDown = false;*/
    }

    public void setPosition(double pos){
        left.setPosition(downL-pos);
        right.setPosition(downR+pos);
    }

    public void setAdjust(double adjust){
        downL -= adjust;
        downR += adjust;

        setDown();
    }

    public void move(double amount){
        amount *= 0.007;

        left.setPosition(left.getPosition() - amount);
        right.setPosition(right.getPosition() + amount);


    }




}
