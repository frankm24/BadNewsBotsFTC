package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        /*
        robot length: 15 in.
        robot width: 14.6 in.
         */

        float robotLength = 15.0f;
        float robotWidth = 14.6f;
        float tileSize = 24.0f;

        Pose2d redStartPose = new Pose2d(-(1.5 * tileSize), 72 - robotWidth/2, Math.toRadians(180));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528, Math.toRadians(261.482587826087), Math.toRadians(261.482587826087), 11.38)
                .setDimensions(13.25, 16.5)
                // Enter traj seq. to follow
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(redStartPose)
                        .forward(tileSize)
                        .addTemporalMarker(() -> {
                            // release cone
                        })
                        .back(5)
                        .lineTo(new Vector2d( -2.5*tileSize + 5, tileSize/2))
                        .forward(6)
                        .addTemporalMarker(() -> {
                            // pick up next cone
                            // tell linear mechanism to begin moving up, rotating as necessary
                        })
                        .setReversed(true)
                        .splineTo(new Vector2d(-1.5*tileSize + 7, 8), Math.toRadians(-55))
                        .setReversed(false)
                        .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setTheme(new ColorSchemeRedDark())
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}