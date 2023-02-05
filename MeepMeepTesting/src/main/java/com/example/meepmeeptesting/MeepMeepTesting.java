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
import com.noahbres.meepmeep.roadrunner.DriveShim;
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
        float tileSize = 23.5f;
        float coneDiameter = 4.0f;

        Pose2d redStartPose1 = new Pose2d(-1.5 * tileSize, -3 * tileSize + robotWidth/2, Math.toRadians(180));
        Pose2d redStartPose2 = new Pose2d(1.5 * tileSize, -3 * tileSize + robotWidth/2, Math.toRadians(180));

        Pose2d blueStartPose1 = new Pose2d((1.5 * tileSize), 3 * tileSize - robotWidth/2, Math.toRadians(0));
        Pose2d blueStartPose2 = new Pose2d(-(1.5 * tileSize), 3 * tileSize - robotWidth/2, Math.toRadians(0));

        Constraints constraints = new Constraints(52.48291908330528, 52.48291908330528,
                Math.toRadians(261.482587826087), Math.toRadians(261.482587826087), 11.38);

        RoadRunnerBotEntity robot = new RoadRunnerBotEntity(meepMeep, constraints, robotWidth, robotLength,
                new Pose2d(), new ColorSchemeBlueLight(), 1, DriveTrainType.MECANUM, true);

        Pose2d center = new Pose2d(blueStartPose2.getX(),
                blueStartPose2.getY() -(2.5*tileSize - robotWidth/2) , Math.toRadians(180));
        Pose2d betweenGoals = center.minus(new Pose2d(12, 0, 0));
        Pose2d atCone = center.plus(new Pose2d(-(coneDiameter + tileSize), 0, 0));
        Pose2d atHighGoal = new Pose2d(-tileSize - 6.9, 6.4, Math.toRadians(-42.5));
        TrajectorySequence blueAuto2_1 = robot.getDrive().trajectorySequenceBuilder(blueStartPose2)
                .strafeRight(2*tileSize)
                .lineToLinearHeading(center)
                .lineToLinearHeading(atCone)
                .setReversed(true)
                .lineToLinearHeading(betweenGoals)
                .splineTo(new Vector2d(atHighGoal.getX(), atHighGoal.getY()), atHighGoal.getHeading())
                .build();

        TrajectorySequence redAutoParking1_1 = robot.getDrive().trajectorySequenceBuilder(redStartPose1)
                .forward(tileSize)
                .strafeRight(1.5*tileSize)
                .build();

        TrajectorySequence redAutoParking1_2 = robot.getDrive().trajectorySequenceBuilder(redStartPose1)
                .strafeRight(2*tileSize)
                .build();

        TrajectorySequence redAutoParking1_3 = robot.getDrive().trajectorySequenceBuilder(redStartPose1)
                .back(tileSize)
                .strafeRight(1.5*tileSize)
                .build();

        TrajectorySequence redAutoParking2_1 = robot.getDrive().trajectorySequenceBuilder(redStartPose2)
                .forward(tileSize)
                .strafeRight(1.5*tileSize)
                .build();

        TrajectorySequence redAutoParking2_2 = robot.getDrive().trajectorySequenceBuilder(redStartPose2)
                .strafeRight(2*tileSize)
                .build();

        TrajectorySequence redAutoParking2_3 = robot.getDrive().trajectorySequenceBuilder(redStartPose2)
                .back(tileSize)
                .strafeRight(1.5*tileSize)
                .build();

        TrajectorySequence blueAutoParking1_1 = robot.getDrive().trajectorySequenceBuilder(blueStartPose1)
                .forward(tileSize)
                .strafeRight(1.5*tileSize)
                .build();

        TrajectorySequence blueAutoParking1_2 = robot.getDrive().trajectorySequenceBuilder(blueStartPose1)
                .strafeRight(2*tileSize)
                .build();

        TrajectorySequence blueAutoParking1_3 = robot.getDrive().trajectorySequenceBuilder(blueStartPose1)
                .back(tileSize)
                .strafeRight(1.5*tileSize)
                .build();

        TrajectorySequence blueAutoParking2_1 = robot.getDrive().trajectorySequenceBuilder(blueStartPose2)
                .forward(tileSize)
                .strafeRight(1.5*tileSize)
                .build();

        TrajectorySequence blueAutoParking2_2 = robot.getDrive().trajectorySequenceBuilder(blueStartPose2)
                .strafeRight(2*tileSize)
                .build();

        TrajectorySequence blueAutoParking2_3 = robot.getDrive().trajectorySequenceBuilder(blueStartPose2)
                .back(tileSize)
                .strafeRight(1.5*tileSize)
                .build();

        robot.followTrajectorySequence(blueAuto2_1);

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setTheme(new ColorSchemeRedDark())
                .setBackgroundAlpha(0.95f)
                .addEntity(robot)
                .start();
    }
}

/*
                .back(tileSize)
                .addTemporalMarker(() -> {
                    //
                })
                .forward(2.5)
                .lineTo(new Vector2d( -2.5*tileSize + 2.5, tileSize/2))
                .lineTo(new Vector2d(-3*tileSize + robotLength/2, tileSize/2))
                .addTemporalMarker(() -> {
                    // pick up next cone
                    // tell linear mechanism to begin moving up, rotating as necessary
                })
                .waitSeconds(0.5)
                .setReversed(false)
                .lineTo(new Vector2d(-2*tileSize, tileSize/2))
                .splineTo(new Vector2d(-tileSize - 6.9, 6.4), Math.toRadians(-42.5))
                .setReversed(true)
                .addTemporalMarker(() -> {

                })
                .waitSeconds(.5)
                .splineTo(new Vector2d(-2*tileSize, tileSize/2), Math.toRadians(180))
                .lineTo(new Vector2d( -2.5*tileSize + 2.5, tileSize/2))
                .addTemporalMarker(() -> {

                })
                .waitSeconds(.5)
                .setReversed(false)
                // end of cycle
                */

/*
                .splineTo(new Vector2d(-1.5*tileSize + 7, robotWidth/2 + 0.75), Math.toRadians(-54))
                .setReversed(true)
                .addTemporalMarker(() -> {

                })
                .waitSeconds(.5)
                .splineTo(new Vector2d( -2.5*tileSize -1, tileSize/2), Math.toRadians(180))
                .addTemporalMarker(() -> {

                })
                .waitSeconds(.5)
                .setReversed(false)
                .splineTo(new Vector2d(-1.5*tileSize + 7, robotWidth/2 + 0.75), Math.toRadians(-54))
                .setReversed(true)
                .addTemporalMarker(() -> {

                })
                .waitSeconds(.5)
                .splineTo(new Vector2d( -2.5*tileSize -1, tileSize/2), Math.toRadians(180))
                .addTemporalMarker(() -> {

                })
                .waitSeconds(.5)
                .setReversed(false)
                .splineTo(new Vector2d(-1.5*tileSize + 7, robotWidth/2 + 0.75), Math.toRadians(-54))
                .setReversed(true)
                .addTemporalMarker(() -> {

                })
                .waitSeconds(.5)
                .splineTo(new Vector2d( -2.5*tileSize -1, tileSize/2), Math.toRadians(180))
                .addTemporalMarker(() -> {

                })
                .waitSeconds(.5)
                .setReversed(false)
                .splineTo(new Vector2d(-1.5*tileSize + 7, robotWidth/2 + 0.75), Math.toRadians(-54))
                .setReversed(true)
                .addTemporalMarker(() -> {

                })
                .waitSeconds(.5)
                .splineTo(new Vector2d( -2.5*tileSize -1, tileSize/2), Math.toRadians(180))
                .waitSeconds(0.5)
                .strafeLeft(tileSize)
                //.splineTo
                 */