package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.jetbrains.annotations.NotNull;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);
        Pose2d redStartPoseWarehouse = new Pose2d(12, -62.25, Math.toRadians(90));
        Pose2d blueStartPoseWarehouse = new Pose2d(12, 62.25, Math.toRadians(-90));
        Pose2d redStartPoseCarousel = new Pose2d(-35, -62.25, Math.toRadians(90));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528, Math.toRadians(261.482587826087), Math.toRadians(261.482587826087), 11.38)
                .setDimensions(13.25, 16.5)
                // Enter traj seq. to follow
                .followTrajectorySequence(drive ->
                        /*
                        drive.trajectorySequenceBuilder(redStartPose)
                                .strafeLeft(23.75)
                                .forward(14.5)
                                .addTemporalMarker(0, () -> {
                                    System.out.println("door 0");
                                })
                                .addTemporalMarker(0.1, () -> {
                                    System.out.println("arm 0.7");
                                })
                                .addTemporalMarker(2.5, () -> {
                                    System.out.println("door 0.65");
                                })
                                .waitSeconds(1)
                                .lineToSplineHeading(new Pose2d(12, -47.75, Math.toRadians(180)))
                                .back(48)
                                .build()
                         */
                        /*
                        .strafeRight(23.75)
                                .forward(14.5)
                                .addTemporalMarker(0, () -> {
                                    System.out.println("door 0");
                                })
                                .addTemporalMarker(0.1, () -> {
                                    System.out.println("arm 0.7");
                                })
                                .addTemporalMarker(2.5, () -> {
                                    System.out.println("door 0.65");
                                })
                                .waitSeconds(1)
                                .lineToSplineHeading(new Pose2d(12, 47.75, Math.toRadians(0)))
                                .forward(48)
                                .build()
                         */
                        drive.trajectorySequenceBuilder(redStartPoseCarousel)
                            .strafeRight(23.75)
                            .forward(14.5)
                            .addTemporalMarker(0, () -> {
                                System.out.println("door 0");
                            })
                            .addTemporalMarker(0.1, () -> {
                                System.out.println("arm 0.8");
                            })
                            .addTemporalMarker(2.5, () -> {
                                System.out.println("door 0.65");
                            })
                            .waitSeconds(1)
                            .addTemporalMarker(4, () -> {
                                System.out.println("arm 0.2");
                            })
                            .lineToSplineHeading(new Pose2d(-58, -58, Math.toRadians(180)))
                            .waitSeconds(3)
                            .strafeRight(24)
                            .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setTheme(new ColorSchemeRedDark())
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}