package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.Constraints;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

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

        Pose2d redStartPose1 = new Pose2d(-1.5 * tileSize, -3 * tileSize + robotWidth/2, Math.toRadians(0));
        Pose2d redStartPose2 = new Pose2d(1.5 * tileSize, -3 * tileSize + robotWidth/2, Math.toRadians(0));

        Pose2d blueStartPose1 = new Pose2d(-(1.5 * tileSize), 3 * tileSize - robotWidth/2, Math.toRadians(180));
        Pose2d blueStartPose2 = new Pose2d((1.5 * tileSize), 3 * tileSize - robotWidth/2, Math.toRadians(180));

        Constraints constraints = new Constraints(52.48291908330528, 52.48291908330528,
                Math.toRadians(261.482587826087), Math.toRadians(261.482587826087), 11.38);

        RoadRunnerBotEntity robot = new RoadRunnerBotEntity(meepMeep, constraints, 13.25, 16.5,
                new Pose2d(), new ColorSchemeBlueLight(), 1, DriveTrainType.MECANUM, true);

        TrajectorySequence blueAuto1 = robot.getDrive().trajectorySequenceBuilder(blueStartPose1)
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
                .build();

        TrajectorySequence redAutoParking1_1 = robot.getDrive().trajectorySequenceBuilder(redStartPose1)
                .setReversed(true)
                .splineTo(new Vector2d(redStartPose1.getX() - tileSize, redStartPose1.getY() + tileSize + robotLength/2),
                        Math.toRadians(90))
                .setReversed(false)
                .build();

        TrajectorySequence redAutoParking1_2 = robot.getDrive().trajectorySequenceBuilder(redStartPose1)
                .setReversed(true)
                .strafeLeft(1 * tileSize + robotWidth/2)
                .build();

        TrajectorySequence redAutoParking1_3 = robot.getDrive().trajectorySequenceBuilder(redStartPose1)
                .splineTo(new Vector2d(redStartPose1.getX() + tileSize, redStartPose1.getY() + tileSize + robotLength/2),
                        Math.toRadians(90))
                .build();

        TrajectorySequence redAutoParking2_1 = robot.getDrive().trajectorySequenceBuilder(redStartPose2)
                .setReversed(true)
                .splineTo(new Vector2d(redStartPose2.getX() - tileSize, redStartPose2.getY() + tileSize + robotLength/2),
                        Math.toRadians(90))
                .setReversed(false)
                .build();

        TrajectorySequence redAutoParking2_2 = robot.getDrive().trajectorySequenceBuilder(redStartPose2)
                .setReversed(true)
                .strafeLeft(1 * tileSize + robotWidth/2)
                .build();

        TrajectorySequence redAutoParking2_3 = robot.getDrive().trajectorySequenceBuilder(redStartPose2)
                .splineTo(new Vector2d(redStartPose2.getX() + tileSize, redStartPose2.getY() + tileSize + robotLength/2),
                        Math.toRadians(90))
                .build();

        TrajectorySequence blueAutoParking1_1 = robot.getDrive().trajectorySequenceBuilder(blueStartPose1)
                .splineTo(new Vector2d(blueStartPose1.getX() - tileSize, blueStartPose1.getY() - tileSize - robotLength/2),
                        Math.toRadians(270))
                .build();

        TrajectorySequence blueAutoParking1_2 = robot.getDrive().trajectorySequenceBuilder(blueStartPose1)
                .setReversed(true)
                .strafeLeft(1 * tileSize + robotWidth/2)
                .build();

        TrajectorySequence blueAutoParking1_3 = robot.getDrive().trajectorySequenceBuilder(blueStartPose1)
                .setReversed(true)
                .splineTo(new Vector2d(blueStartPose2.getX() + tileSize, blueStartPose2.getY() - tileSize - robotLength/2),
                        Math.toRadians(270))
                .setReversed(false)
                .build();

        TrajectorySequence blueAutoParking2_1 = robot.getDrive().trajectorySequenceBuilder(blueStartPose2)
                .splineTo(new Vector2d(blueStartPose2.getX() - tileSize, blueStartPose2.getY() - tileSize - robotLength/2),
                        Math.toRadians(270))
                .build();

        TrajectorySequence blueAutoParking2_2 = robot.getDrive().trajectorySequenceBuilder(blueStartPose2)
                .setReversed(true)
                .strafeLeft(1 * tileSize + robotWidth/2)
                .build();

        TrajectorySequence blueAutoParking2_3 = robot.getDrive().trajectorySequenceBuilder(blueStartPose2)
                .setReversed(true)
                .splineTo(new Vector2d(blueStartPose2.getX() + tileSize, blueStartPose2.getY() - tileSize - robotLength/2),
                        Math.toRadians(270))
                .setReversed(false)
                .build();


        robot.followTrajectorySequence(blueAutoParking2_1);

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setTheme(new ColorSchemeRedDark())
                .setBackgroundAlpha(0.95f)
                .addEntity(robot)
                .start();
    }
}