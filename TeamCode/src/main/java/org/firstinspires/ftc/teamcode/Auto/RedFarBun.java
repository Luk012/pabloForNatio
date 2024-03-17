package org.firstinspires.ftc.teamcode.Auto;



import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Auto.AutoControllers.RedFarAutoController;
import org.firstinspires.ftc.teamcode.Auto.Recognition.RedPipelineStackMaster;
import org.firstinspires.ftc.teamcode.Auto.Recognition.YellowPixelMaster;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.opmode.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.globals.robotMap;

import org.firstinspires.ftc.teamcode.system_controllers.clawAngleController;
import org.firstinspires.ftc.teamcode.system_controllers.clawFlipController;
import org.firstinspires.ftc.teamcode.system_controllers.collectAngleController;
import org.firstinspires.ftc.teamcode.system_controllers.doorController;
import org.firstinspires.ftc.teamcode.system_controllers.droneController;
import org.firstinspires.ftc.teamcode.system_controllers.extendoController;
import org.firstinspires.ftc.teamcode.system_controllers.fourbarController;
import org.firstinspires.ftc.teamcode.system_controllers.latchLeftController;
import org.firstinspires.ftc.teamcode.system_controllers.latchRightController;
import org.firstinspires.ftc.teamcode.system_controllers.liftController;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;



import java.util.List;

@Config
@Autonomous(group = "Auto" , name = "RedFar")

public class RedFarBun extends LinearOpMode {

    enum STROBOT {
        START,
        PURPLE,
        COLLECT_PURPLE,
        VERIF_TIMER,
        GO_SCORE_YELLOW,
        PREPARE_SCORE_YELLOW,
        VERIF_PURPLE_SCORE,
        YELLOW_DROP,

        PREPARE_COLLECT,
        COLLECT_EXTENDO,
        COLLECT_VERIF_PIXLES,
        COLLECT_VERIF_PIXLES_V2,

        GO_SCORE_CYCLE,
        PREPARE_SCORE_CYCLE,
        SCORE_CYCLE,

        NOTHING
    }

    public static double x_start = -43, y_start = -61, angle_start = 270;


    /**
     * purple
     */

    public static double x_purple_preload_right = -40, y_purple_preload_right = -24, angle_purple_preload_right = 171.5;
    public static double x_purple_preload_center = -54, y_purple_preload_center = -26, angle_purple_preload_center = 177;
    public static double x_purple_preload_left = -62, y_purple_preload_left = -26, angle_purple_preload_left = 170;

    /**
     * yellow
     */

    public static double x_yellow_preload_right = 41, y_yellow_preload_right = -49, angle_yellow_preload_right = 182;
    public static double x_yellow_preload_center = 41, y_yellow_preload_center = -29, angle_yellow_preload_center = 180;
    public static double x_yellow_preload_left = 41, y_yellow_preload_left = -25, angle_yellow_preload_left = 180;

    /**
     * collect
     */

    public static double x_collect = -19, y_collect = -7, angle_collect = 180;

    public static double x_collect_middle = -20, y_collect_middle = -6.5, angle_collect_middle = 195;


    public static double x_inter_collect_cycle_2 = 30, y_inter_collect_cycle_2 = -2, angle_inter_collect_cycle_2 = 180;
    public static double x_collect_cycle_2 = -23.5, y_collect_cycle_2 = -5, angle_collect_cycle_2 = 180;

    public static double x_inter_collect_cycle_3 = 30, y_inter_collect_cycle_3 = -10.5, angle_inter_collect_cycle_3 = 182;
    public static double x_collect_cycle_3 = -26, y_collect_cycle_3 = -10.5, angle_collect_cycle_3 = 182;

    /**
     * score
     */


    public static double x_score = 47, y_score = -10, angle_score = 150;

    public static double x_score_second_cycle = 45, y_score_second_cycle = -6, angle_score_second_angle = 140;
    public static double x_score_third_cycle = 45, y_score_third_cycle = -10, angle_score_third_angle = 150;



    /**
     * intern collect
     */

    public static double x_inter_collect_first_cycle = -36, y_inter_collect_first_cycle = 0, angle_inter_collect_first_cycle = 180;

    /**
     * intern score
     */

    public static double x_inter_score_first_cycle = 30, y_inter_score_first_cycle = -7.5, angle_inter_score_first_cycle =180;

    int caz = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        RedPipelineStackMaster redLeft = new RedPipelineStackMaster(this);
        redLeft.observeStick();




        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robotMap r = new robotMap(hardwareMap);


        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);


        clawAngleController clawAngle = new clawAngleController();
        clawFlipController clawFlip = new clawFlipController();
        collectAngleController collectAngle = new collectAngleController();
        doorController door = new doorController();
        fourbarController fourbar = new fourbarController();
        latchLeftController latchLeft = new latchLeftController();
        latchRightController latchRight = new latchRightController();
        droneController drone = new droneController();
        liftController lift = new liftController();
        extendoController extendo = new extendoController();

        RedFarAutoController redFarAutoController = new RedFarAutoController();


        double voltage;
        VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        voltage = batteryVoltageSensor.getVoltage();

        clawAngle.CS = clawAngleController.clawAngleStatus.COLLECT;
        clawFlip.CS = clawFlipController.clawFlipStatus.DRIVE;
        collectAngle.CS = collectAngleController.collectAngleStatus.INITIALIZE;
        door.CS = doorController.doorStatus.CLOSED;
        fourbar.CS = fourbarController.fourbarStatus.DRIVE;
        latchLeft.CS = latchLeftController.LatchLeftStatus.SECURED;
        latchRight.CS = latchRightController.LatchRightStatus.SECURED;
        lift.CS = liftController.liftStatus.DOWN;
        extendo.CS = extendoController.extendoStatus.RETRACTED;

//
        drive.update();
        clawAngle.update(r);
        clawFlip.update(r);
        door.update(r);
        fourbar.update(r);
        latchLeft.update(r);
        latchRight.update(r);
        drone.update(r);
        lift.update(r, 0, voltage);
        extendo.update(r, 0, 1, voltage);
        redFarAutoController.update(r, lift, fourbar,clawAngle,clawFlip,collectAngle,door,extendo,latchLeft,latchRight);

        collectAngle.update(r);



        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        }

        Pose2d start_pose = new Pose2d(x_start, y_start,Math.toRadians(angle_start));

        Pose2d purpleRight = new Pose2d(x_purple_preload_right, y_purple_preload_right, Math.toRadians(angle_purple_preload_right));
        Pose2d purpleCenter = new Pose2d(x_purple_preload_center, y_purple_preload_center, Math.toRadians(angle_purple_preload_center));
        Pose2d purpleLeft = new Pose2d(x_purple_preload_left, y_purple_preload_left, Math.toRadians(angle_purple_preload_left));
         
        Pose2d interCollectFirstCycle = new Pose2d(x_inter_collect_first_cycle, y_inter_collect_first_cycle, Math.toRadians(angle_inter_collect_first_cycle));
        Pose2d interScoreFirstCycle = new Pose2d(x_inter_score_first_cycle, y_inter_score_first_cycle, Math.toRadians(angle_inter_score_first_cycle));

        Pose2d yellowRight = new Pose2d(x_yellow_preload_right, y_yellow_preload_right, Math.toRadians(angle_yellow_preload_right));
        Pose2d yellowCenter = new Pose2d(x_yellow_preload_center, y_yellow_preload_center, Math.toRadians(angle_yellow_preload_center));
        Pose2d yellowLeft = new Pose2d(x_yellow_preload_left, y_yellow_preload_left, Math.toRadians(angle_yellow_preload_left));

        Pose2d collect_inter_cycle2 = new Pose2d(x_inter_collect_cycle_2, y_inter_collect_cycle_2, Math.toRadians(angle_inter_collect_cycle_2));
        Pose2d collect_cycle2 = new Pose2d(x_collect_cycle_2, y_collect_cycle_2, Math.toRadians(angle_collect_cycle_2));

        Pose2d collect_inter_cycle3 = new Pose2d(x_inter_collect_cycle_3, y_inter_collect_cycle_3, Math.toRadians(angle_inter_collect_cycle_3));
        Pose2d collect_cycle3 = new Pose2d(x_collect_cycle_3, y_collect_cycle_3, Math.toRadians(angle_collect_cycle_3));

        Pose2d score_second_cycle = new Pose2d(x_score_second_cycle, y_score_second_cycle, Math.toRadians(angle_score_second_angle));
        Pose2d score_third_cycle = new Pose2d(x_score_third_cycle, y_score_third_cycle, Math.toRadians(angle_score_third_angle));

        TrajectorySequence PURPLE_RIGHT = drive.trajectorySequenceBuilder(start_pose)
                .lineToLinearHeading(purpleRight)
                .build();

        TrajectorySequence PURPLE_CENTER = drive.trajectorySequenceBuilder(start_pose)
                .lineToLinearHeading(purpleCenter)
                .build();

        TrajectorySequence PURPLE_LEFT = drive.trajectorySequenceBuilder(start_pose)
                .lineToLinearHeading(purpleLeft)
                .build();
        
        TrajectorySequence YELLOW_LEFT = drive.trajectorySequenceBuilder(purpleLeft)
                .lineToLinearHeading(interCollectFirstCycle)
                .lineToLinearHeading(interScoreFirstCycle)
                .lineToLinearHeading(yellowLeft)
                .build();

        TrajectorySequence YELLOW_CENTER = drive.trajectorySequenceBuilder(purpleCenter)
                .lineToLinearHeading(interCollectFirstCycle)
                .lineToLinearHeading(interScoreFirstCycle)
                .lineToLinearHeading(yellowCenter)
                .build();

        TrajectorySequence YELLOW_RIGHT = drive.trajectorySequenceBuilder(purpleRight)
                .lineToLinearHeading(interCollectFirstCycle)
                .lineToLinearHeading(interScoreFirstCycle)
                .lineToLinearHeading(yellowRight)
                .build();


        TrajectorySequence YELLOW_DROP = drive.trajectorySequenceBuilder(yellowRight)
                        .back(12)
                                .build();

        TrajectorySequence YELLOW_DROP_V2 = drive.trajectorySequenceBuilder(yellowRight)
                .forward(8)
                .build();

        TrajectorySequence COLLECT_CYCLE_2 = drive.trajectorySequenceBuilder(yellowRight)
                        .lineToLinearHeading(collect_inter_cycle2)
                                .lineToLinearHeading(collect_cycle2)
                                        .build();

        TrajectorySequence COLLECT_CYCLE_3 = drive.trajectorySequenceBuilder(score_second_cycle)
                .lineToLinearHeading(collect_inter_cycle3)
                .lineToLinearHeading(
                        collect_cycle3
                        )
                .build();


        TrajectorySequence SCORE_SECOND_CYCLE = drive.trajectorySequenceBuilder(collect_cycle2)
                        .lineToLinearHeading(score_second_cycle)
                                .build();

        TrajectorySequence SCORE_THIRD_CYCLE = drive.trajectorySequenceBuilder(collect_cycle3)
                .lineToLinearHeading(score_third_cycle)
                .build();

        drive.setPoseEstimate(start_pose);
        STROBOT status = STROBOT.START;


        int nrcicluri = 0;
        double loopTime = 0;

        ElapsedTime transfer = new ElapsedTime();
        ElapsedTime prepare_score_yellowqe = new ElapsedTime();
        ElapsedTime prepare_collect = new ElapsedTime();
        ElapsedTime extendo_timer = new ElapsedTime();
        ElapsedTime verif_pixels = new ElapsedTime();
        ElapsedTime verif = new ElapsedTime();
        ElapsedTime score = new ElapsedTime();

        extendo.caz = 0;
        extendo.cycle_i = 0;
        collectAngle.collectAngle_i = 4;
        lift.i_up = 0;


        while (!isStarted() && !isStopRequested()) {

            sleep(20);
            if(redLeft.opencvred.getWhichSide() == "left"){
                caz = 0;
            } else if (redLeft.opencvred.getWhichSide() == "center") {
                caz = 1;
            } else {
                caz = 2;
            }
            telemetry.addData("case", redLeft.opencvred.getWhichSide());
            telemetry.update();
            sleep(50);
        }

        waitForStart();


        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            int position = r.lift.getCurrentPosition();
            int extendopos = r.extendoLeft.getCurrentPosition();

            //  extendo.distance= r.extendoDistance.getDistance(DistanceUnit.MM);

            switch (status) {

                case START: {

                    switch (caz)
                    {
                        case 0:
                        {
                            drive.followTrajectorySequenceAsync(PURPLE_LEFT);
                            extendo.caz = 2;
                            break;
                        }
                        case 1:
                        {
                            drive.followTrajectorySequenceAsync(PURPLE_CENTER);
                            extendo.caz = 1;
                            break;

                        }
                        case 2:
                        {
                            drive.followTrajectorySequenceAsync(PURPLE_RIGHT);
                            extendo.caz = 0;
                                break;
                        }
                    }

                    redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.PURPLE;
                    extendo_timer.reset();
                    status = STROBOT.PURPLE;
                    break;
                }

                case PURPLE:
                {

                    if(!drive.isBusy())
                    {
                        r.collect.setPower(1);
                        extendo.CS = extendoController.extendoStatus.PURPLE;
                        collectAngle.CS = collectAngleController.collectAngleStatus.COLLECT;
                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.PURPLE_DROP;
                        status = STROBOT.COLLECT_PURPLE;
                    }
                    break;
                }

                case COLLECT_PURPLE:
                {
                    if(!r.pixelLeft.getState() && !r.pixelRight.getState())
                    {
                      verif.reset();
                      status = STROBOT.VERIF_TIMER;
                        }
                    break;

                }

                case VERIF_TIMER:
                {
                    if (verif.seconds() > 0.5)
                    { collectAngle.CS = collectAngleController.collectAngleStatus.DRIVE;
                    extendo.CS = extendoController.extendoStatus.RETRACTED;
                    r.collect.setPower(-0.5);
                    transfer.reset();
                    status = STROBOT.GO_SCORE_YELLOW;
                    }
                    break;
                }
                
                case GO_SCORE_YELLOW:
                {

                    switch (caz)
                    {
                        case 0:
                        {
                            drive.followTrajectorySequenceAsync(YELLOW_LEFT);
                            break;
                        }
                        case 1:
                        {
                            drive.followTrajectorySequenceAsync(YELLOW_CENTER);
                            break;

                        }
                        case 2:
                        {
                            drive.followTrajectorySequenceAsync(YELLOW_RIGHT);
                            break;
                        }
                    }

                    if(transfer.seconds() > 0.5)
                    {
                        r.collect.setPower(0);
                    }

                    if(transfer.seconds() > 1.2)
                    {
                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.TRANSFER_BEGIN;
                        prepare_score_yellowqe.reset();
                        status = STROBOT.PREPARE_SCORE_YELLOW;
                    }
                    break;

                }

                case PREPARE_SCORE_YELLOW:
                {
                    if(drive.getPoseEstimate().getX() >= 15 && redFarAutoController.CurrentStatus == RedFarAutoController.autoControllerStatus.TRANSFER_DONE)
                    {
                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.SCORE_YELLOW_BEGIN;
                        status = STROBOT.VERIF_PURPLE_SCORE;
                    }

                    break;
                }

                case VERIF_PURPLE_SCORE:
                {
                    if(!drive.isBusy() && redFarAutoController.CurrentStatus == RedFarAutoController.autoControllerStatus.SCORE_YELLOW_DONE)
                    {
                        drive.followTrajectorySequenceAsync(YELLOW_DROP);
                        status = STROBOT.YELLOW_DROP;
                    }
                    break;
                }

                case YELLOW_DROP:
                {
                    if(r.back.getDistance(DistanceUnit.CM) < 20)
                    {
                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.LATCH_DROP;

                                drive.followTrajectorySequenceAsync(COLLECT_CYCLE_2);

                        prepare_collect.reset();
                        status= STROBOT.PREPARE_COLLECT;
                    }
                    break;
                }

                case PREPARE_COLLECT:
                {
                    if(prepare_collect.seconds() > 0.2)
                    {
                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.COLLECT_PREPARE;
                        extendo_timer.reset();
                        status = STROBOT.COLLECT_EXTENDO;
                    }
                    break;
                }

                case COLLECT_EXTENDO:
                {
                    if(extendo_timer.seconds() > 4)
                    {
                        collectAngle.CS = collectAngleController.collectAngleStatus.COLLECT;
                        r.collect.setPower(1);

                        switch (nrcicluri)
                        {
                            case 0:
                            {
                                extendo.cycle_i = 0;
                                collectAngle.collectAngle_i = 4;
                                extendo.CS = extendoController.extendoStatus.CYCLE;
                                break;
                            }
                            case 1:
                            {
                                extendo.cycle_i = 1;
                                collectAngle.collectAngle_i = 2;
                                extendo.CS = extendoController.extendoStatus.CYCLE;
                                break;
                            }
                            case 2:
                            {
                                collectAngle.collectAngle_i = 3;
                                extendo.cycle_i = 2;
                                extendo.CS = extendoController.extendoStatus.CYCLE;
                                break;
                            }

                        }
                        status = STROBOT.COLLECT_VERIF_PIXLES;
                    }
                    break;
                }

                case COLLECT_VERIF_PIXLES:
                {
                    if(!r.pixelLeft.getState() || !r.pixelRight.getState())
                    {
                              collectAngle.collectAngle_i -=1;
                              status = STROBOT.COLLECT_VERIF_PIXLES_V2;
                    }
                    break;
                }

                case COLLECT_VERIF_PIXLES_V2:
                {
                    if(!r.pixelLeft.getState() && !r.pixelRight.getState())
                    {
                        r.collect.setPower(-0.5);
                        extendo.CS = extendoController.extendoStatus.RETRACTED;
                        status = STROBOT.GO_SCORE_CYCLE;
                    }
                    break;
                }

                case GO_SCORE_CYCLE:
                {
                    r.collect.setPower(0);
                    switch (nrcicluri)
                    {
                        case 0:
                        {
                            drive.followTrajectorySequenceAsync(SCORE_SECOND_CYCLE);
                            break;
                        }

                        case 1:
                        {
                            drive.followTrajectorySequenceAsync(SCORE_THIRD_CYCLE);
                            break;
                        }
                    }
                    redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.TRANSFER_BEGIN;
                    status = STROBOT.PREPARE_SCORE_CYCLE;
                    break;
                }

                case PREPARE_SCORE_CYCLE:
                {
                    if(redFarAutoController.CurrentStatus == RedFarAutoController.autoControllerStatus.TRANSFER_DONE)
                    {
                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.SCORE_YELLOW_BEGIN;
                        nrcicluri +=1;
                        score.reset();
                        status = STROBOT.SCORE_CYCLE;
                    }
                    break;
                }

                case SCORE_CYCLE:
                {
                    if(!drive.isBusy() && redFarAutoController.CurrentStatus == RedFarAutoController.autoControllerStatus.SCORE_YELLOW_DONE && score.seconds() > 1)
                    {
                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.LATCH_DROP;

                        switch (nrcicluri)
                        {
                            case 1:
                            {
                                drive.followTrajectorySequenceAsync(COLLECT_CYCLE_3);
                                break;
                            }

                        }
                        prepare_collect.reset();
                        status= STROBOT.PREPARE_COLLECT;
                    }

                    break;
                }


            }

            drive.update();
            clawAngle.update(r);
            clawFlip.update(r);
            door.update(r);
            fourbar.update(r);
            latchLeft.update(r);
            latchRight.update(r);
            drone.update(r);
            lift.update(r, position, voltage);
            extendo.update(r, extendopos, 1, voltage);
            collectAngle.update(r);
            redFarAutoController.update(r, lift, fourbar,clawAngle,clawFlip,collectAngle,door,extendo,latchLeft,latchRight);



//            telemetry.addData("status", status);
//            telemetry.addData("fourbar", fourbar.CS);
//            telemetry.addData("flip", clawFlip.CS);
//            telemetry.addData("angle", clawAngle.CS);
//            telemetry.addData("status autocontroller", redFarAutoController.CurrentStatus);
            telemetry.addData("heading", drive.getPoseEstimate().getY());
           // telemetry.addData("distance", r.back.getDistance(DistanceUnit.CM));
            double loop = System.nanoTime();

            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
          //  telemetry.addData("position", extendopos);
            //   telemetry.addData("target", extendo.target);
            //    telemetry.addData("distance", r.extendoDistance.getDistance(DistanceUnit.MM));

            loopTime = loop;

            telemetry.update();

        }

    }
}
