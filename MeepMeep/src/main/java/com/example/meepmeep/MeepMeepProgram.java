package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepProgram {



    public static void main(String[] args){
        System.setProperty("sun.java2d.opengl", "true");

        MeepMeep mm = new MeepMeep(600);


        int hubX = 25;
        int hubY = 15;


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(mm)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 10.25)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(2, -20))
                                //.splineTo(new Vector2d(20, -80), 0)
                                .build()
                );
        myBot.setDimensions(13, 13);



        mm.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                //.removeEntity(MeepMeep.DEFAULT_AXES_ENTITY)
                //.removeEntity(MeepMeep.DEFAULT_COMPASS_ENTITY)
                .addEntity(myBot)
                .start();
    }
}