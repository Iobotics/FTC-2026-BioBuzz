package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Image customFieldImage = null;
        try {
            customFieldImage = ImageIO.read(new File("C:\\Users\\roboticslab\\Downloads\\BioBuzz.png"));
        } catch (IOException e) {
            e.printStackTrace();
        }

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-61, 18, 0))

                        .forward(9)
                        .strafeLeft(12)
                        .lineToConstantHeading(new Vector2d(30, 59))

                        
                     //   .turn(Math.toRadians(90))
                     //   .forward(30)
                        // .turn(Math.toRadians(-90))
                        //.forward(76)
                        //.turn(Math.toRadians(90))
                        //.forward(10)

                        .build());


        meepMeep.setBackground(MeepMeep.Background.GRID_BLUE)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot);
                meepMeep
                        .setBackground(customFieldImage)
                .start();
    }
}