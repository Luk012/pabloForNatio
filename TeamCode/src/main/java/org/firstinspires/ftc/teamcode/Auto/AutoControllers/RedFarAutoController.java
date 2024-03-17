package org.firstinspires.ftc.teamcode.Auto.AutoControllers;


import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.globals.robotMap;

import org.firstinspires.ftc.teamcode.system_controllers.clawAngleController;
import org.firstinspires.ftc.teamcode.system_controllers.clawFlipController;
import org.firstinspires.ftc.teamcode.system_controllers.collectAngleController;
import org.firstinspires.ftc.teamcode.system_controllers.doorController;
import org.firstinspires.ftc.teamcode.system_controllers.extendoController;
import org.firstinspires.ftc.teamcode.system_controllers.fourbarController;
import org.firstinspires.ftc.teamcode.system_controllers.latchLeftController;
import org.firstinspires.ftc.teamcode.system_controllers.latchRightController;
import org.firstinspires.ftc.teamcode.system_controllers.liftController;


public class RedFarAutoController {
    public enum autoControllerStatus
    {
        NOTHING,
        PURPLE,
        PURPLE_DONE,

        PURPLE_DROP,
        PURPLE_DRIVE,
        PURPLE_DROP_DONE,

        TRANSFER_BEGIN,
        TRANSFER_FOURBAR,
        TRANSFER_CLAW,
        TRANSFER_LATCHES,
        TRANSFER_DRIVE_POSE,
        TRANSFER_DONE,

        SCORE_YELLOW_BEGIN,
        TIMER_RESET,
        SCORE_YELLOW_LIFT,
        SCORE_YELLOW_CLAW,
        SCORE_YELLOW_DONE,

        COLLECT_PREPARE,
        COLLECT_PREPARE_CLAW,
        COLLECT_PREPARE_LIFT,
        COLLECT_PREPARE_DONE,

        SCORE_CYCLE_BEGIN,
        TIMER_CYCLE_RESET,
        SCORE_CYCLE_LIFT,
        SCORE_CYCLE_CLAW,
        SCORE_CYCLE_DONE,


        LATCH_DROP,


    }
    public static autoControllerStatus CurrentStatus = autoControllerStatus.NOTHING, PreviousStatus = autoControllerStatus.NOTHING;

    ElapsedTime purple_drive = new ElapsedTime();
    ElapsedTime fourbar_timer = new ElapsedTime();
    ElapsedTime claw_timer = new ElapsedTime();
    ElapsedTime latches_timer = new ElapsedTime();
    ElapsedTime pulamea = new ElapsedTime();


    public void update(robotMap r, liftController lift, fourbarController fourbar, clawAngleController clawAngle, clawFlipController clawFlip, collectAngleController collectAngle, doorController door, extendoController extendo, latchLeftController latchLeft, latchRightController latchRight)
    {
        switch (CurrentStatus)
        {

            /**
             * PURPLE
             */

            case PURPLE:
            {
                fourbar.CS = fourbarController.fourbarStatus.PRELOAD;
                clawFlip.CS = clawFlipController.clawFlipStatus.PURPLE;
                CurrentStatus = autoControllerStatus.PURPLE_DONE;
                break;
            }
            case PURPLE_DROP:
            {
                latchLeft.CS = latchLeftController.LatchLeftStatus.CLOSED;
                latchRight.CS = latchRightController.LatchRightStatus.CLOSED;
                purple_drive.reset();
                CurrentStatus = autoControllerStatus.PURPLE_DRIVE;
                break;
            }

            case PURPLE_DRIVE:
            {
                if(purple_drive.seconds() > 0.2)
                { fourbar.CS = fourbarController.fourbarStatus.DRIVE;
                clawFlip.CS = clawFlipController.clawFlipStatus.DRIVE;
                CurrentStatus = autoControllerStatus.PURPLE_DROP_DONE;
                }
                break;
            }

            /**
             * TRANSFER
             */

            case TRANSFER_BEGIN:
            {
                if(r.extendoLeft.getCurrentPosition() < 5)
                {
                    door.CS = doorController.doorStatus.OPENED;
                    fourbar_timer.reset();
                    CurrentStatus = autoControllerStatus.TRANSFER_FOURBAR;
                }
                break;
            }

            case TRANSFER_FOURBAR:
            {
                if(fourbar_timer.seconds() > 0.4)
                {
                    latchLeft.CS = latchLeftController.LatchLeftStatus.CLOSED;
                    latchRight.CS = latchRightController.LatchRightStatus.CLOSED;
                    fourbar.CS = fourbarController.fourbarStatus.COLLECT;
                   CurrentStatus = autoControllerStatus.TRANSFER_CLAW;
                }
                break;
            }

            case TRANSFER_CLAW:
            {
                clawFlip.CS = clawFlipController.clawFlipStatus.COLLECT;
                clawAngle.CS = clawAngleController.clawAngleStatus.COLLECT;
                latches_timer.reset();
                CurrentStatus = autoControllerStatus.TRANSFER_LATCHES;
                break;
            }

            case TRANSFER_LATCHES:
            {
                if(latches_timer.seconds() > 0.65)
                {
                    latchLeft.CS = latchLeftController.LatchLeftStatus.SECURED;
                    latchRight.CS = latchRightController.LatchRightStatus.SECURED;
                    fourbar_timer.reset();
                    CurrentStatus = autoControllerStatus.TRANSFER_DRIVE_POSE;
                }
                break;
            }

            case TRANSFER_DRIVE_POSE:
            {
                if(fourbar_timer.seconds() > 0.7)
                {
                    fourbar.CS = fourbarController.fourbarStatus.DRIVE;
                }
                if(fourbar_timer.seconds() > 0.715)
                {
                    clawFlip.CS = clawFlipController.clawFlipStatus.DRIVE;
                }
                if(fourbar_timer.seconds() > 0.85)
                {
                  CurrentStatus = autoControllerStatus.TRANSFER_DONE;
                }
                break;

            }

            /**
             * SCORE YELLOW
             */

            case SCORE_YELLOW_BEGIN:
            {
                fourbar.CS = fourbarController.fourbarStatus.SCORE;
                clawFlip.CS = clawFlipController.clawFlipStatus.SCORE;
                CurrentStatus = autoControllerStatus.TIMER_RESET;
                break;
            }

            case TIMER_RESET:
            {
                pulamea.reset();
                CurrentStatus = autoControllerStatus.SCORE_YELLOW_LIFT;
                break;
            }

            case SCORE_YELLOW_LIFT:
            {
                if(pulamea.seconds() > 0.3)
                {
                    door.CS = doorController.doorStatus.CLOSED;
                    lift.pid = 1;
                    lift.CS = liftController.liftStatus.PRELOAD_YELLOW;
                    claw_timer.reset();
                    CurrentStatus = autoControllerStatus.SCORE_YELLOW_CLAW;
                }
                break;
            }

            case SCORE_YELLOW_CLAW:
            {
                if(claw_timer.seconds() > 0.2)
                {
                    clawAngle.clawAngle_i = 3;
                    clawAngle.CS = clawAngleController.clawAngleStatus.SCORE;
                    CurrentStatus = autoControllerStatus.SCORE_YELLOW_DONE;
                }
                break;
            }

            case LATCH_DROP:
            {
                latchLeft.CS = latchLeftController.LatchLeftStatus.CLOSED;
                latchRight.CS = latchRightController.LatchRightStatus.CLOSED;
                        CurrentStatus = autoControllerStatus.NOTHING;
                        break;
            }

            /**
             * PREPARECOLLECT
             */

            case COLLECT_PREPARE:
            {
                fourbar.CS = fourbarController.fourbarStatus.DRIVE;
                claw_timer.reset();
                CurrentStatus = autoControllerStatus.COLLECT_PREPARE_CLAW;
                break;
            }

            case COLLECT_PREPARE_CLAW:
            {
                if(claw_timer.seconds() > 0.1)
                {
                    clawAngle.CS = clawAngleController.clawAngleStatus.COLLECT;
                }
                if(claw_timer.seconds() > 0.2)
                {
                    clawFlip.CS = clawFlipController.clawFlipStatus.DRIVE;
                    pulamea.reset();
                    CurrentStatus = autoControllerStatus.COLLECT_PREPARE_LIFT;
                }
                break;
            }

            case COLLECT_PREPARE_LIFT:
            {
                if(pulamea.seconds() > 0.2)
                {
                    lift.pid = 0;
                    lift.CS = liftController.liftStatus.DOWN;
                    CurrentStatus = autoControllerStatus.COLLECT_PREPARE_DONE;
                }
                break;
            }

            /**
             * SCORE_CYCLE
             */

            case SCORE_CYCLE_BEGIN:
            {
                fourbar.CS = fourbarController.fourbarStatus.SCORE;
                clawFlip.CS = clawFlipController.clawFlipStatus.SCORE;
                CurrentStatus = autoControllerStatus.TIMER_CYCLE_RESET;
                break;
            }

            case TIMER_CYCLE_RESET:
            {
                pulamea.reset();
                CurrentStatus = autoControllerStatus.SCORE_CYCLE_LIFT;
                break;
            }

            case SCORE_CYCLE_LIFT:
            {
                if(pulamea.seconds() > 0.2)
                {
                    door.CS = doorController.doorStatus.CLOSED;
                    lift.pid = 1;
                    lift.CS = liftController.liftStatus.CYCLE;
                    claw_timer.reset();
                    CurrentStatus = autoControllerStatus.SCORE_CYCLE_CLAW;
                }
                break;
            }

            case SCORE_CYCLE_CLAW:
            {
                if(claw_timer.seconds() > 0.1)
                {
                    clawAngle.clawAngle_i = 5;
                    clawAngle.CS = clawAngleController.clawAngleStatus.SCORE;
                    CurrentStatus = autoControllerStatus.SCORE_CYCLE_DONE;
                }
                break;
            }



        }
        PreviousStatus = CurrentStatus;
    }
}

