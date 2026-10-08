package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import java.awt.Image;
import java.io.File;
import java.io.IOException;
import javax.imageio.ImageIO;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
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
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-47, 61, -1.57))
                        .turn(Math.toRadians(90))
                        .strafeTo(new Vector2d(-47, 11.5))
                        .waitSeconds(1)
                        .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                            //outtake
                        })
                        .turn(Math.toRadians(90))
                        .strafeTo(new Vector2d(-61, 11.5))
                        .UNSTABLE_addDisplacementMarkerOffset(40, () -> {
                            //intake
                        })
                        .strafeTo(new Vector2d(-61, 61))
                        .build());

        meepMeep.setBackground(MeepMeep.Background.GRID_BLUE)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot);
        meepMeep
                // Set the background to your loaded custom image
                .setBackground(customFieldImage)
                .start();
    }
}