package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(75, 75, Math.toRadians(240), Math.toRadians(240), 13.5)
                .setDimensions(16, 16)
                .build();

        // Coordinates for the stacks
        Vector2d stack1 = new Vector2d(12, 48);
        Vector2d stack2 = new Vector2d(36, 48);
        Vector2d stack3 = new Vector2d(-11.3, 48);

        // Simulation for RED Alliance, GPP Pattern (Order: 2, 3, 1)
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(60, 0, Math.toRadians(180)))
                // 1. INITIAL SHOT (Preloads)
                .splineTo(new Vector2d(-17, 0), Math.toRadians(180))
                .waitSeconds(0.4) 

                // 2. CYCLE 1 (Stack 2)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(stack2, Math.toRadians(90)), Math.toRadians(90))
                .waitSeconds(0.5) 
                .setTangent(Math.toRadians(225))
                .splineToLinearHeading(new Pose2d(-17, 12, Math.toRadians(180)), Math.toRadians(180))
                .waitSeconds(0.2) 

                // 3. CYCLE 2 (Stack 3)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(stack3, Math.toRadians(90)), Math.toRadians(90))
                .waitSeconds(0.5) 
                .setTangent(Math.toRadians(225))
                .splineToLinearHeading(new Pose2d(-17, 0, Math.toRadians(180)), Math.toRadians(180))
                .waitSeconds(0.2) 

                // 4. HIT LEVER & RACK
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(0, 55, Math.toRadians(90)), Math.toRadians(90))
                .waitSeconds(0.2)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(15, 60, Math.toRadians(90)), Math.toRadians(0))
                .waitSeconds(0.8) 
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-17, 24, Math.toRadians(180)), Math.toRadians(180))
                .waitSeconds(0.4) 

                // 5. CYCLE 3 (Stack 1 - The Green Ball)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(stack1, Math.toRadians(90)), Math.toRadians(90))
                .waitSeconds(0.5) 
                .setTangent(Math.toRadians(225))
                .splineToLinearHeading(new Pose2d(-17, 20, Math.toRadians(180)), Math.toRadians(180))
                .waitSeconds(0.2)

                .splineToLinearHeading(new Pose2d(0, 10, Math.toRadians(180)), Math.toRadians(180))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
