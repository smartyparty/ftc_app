/*
 * Copyright (c) 2017 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import android.speech.tts.TextToSpeech;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

class CmdAutoFull implements TrcRobot.RobotCommand
{
    private static final boolean debugXPid = true;
    private static final boolean debugYPid = true;
    private static final boolean debugTurnPid = true;

    private static final double RED_NEAR_LEFT_COL_OFFSET_IN = -11.5;
    private static final double RED_NEAR_CENTER_COL_OFFSET_IN = -4.0;
    private static final double RED_NEAR_RIGHT_COL_OFFSET_IN = 3.5;

    private static final double RED_FAR_LEFT_COL_OFFSET_IN = -18.0;
    private static final double RED_FAR_CENTER_COL_OFFSET_IN = -10.5;
    private static final double RED_FAR_RIGHT_COL_OFFSET_IN = -3.0;

    private static final double BLUE_NEAR_LEFT_COL_OFFSET_IN = 8.5;
    private static final double BLUE_NEAR_CENTER_COL_OFFSET_IN = 16.0;
    private static final double BLUE_NEAR_RIGHT_COL_OFFSET_IN = 23.5;

    private static final double BLUE_FAR_LEFT_COL_OFFSET_IN = 6.0;
    private static final double BLUE_FAR_CENTER_COL_OFFSET_IN = 13.5;
    private static final double BLUE_FAR_RIGHT_COL_OFFSET_IN = 21.0;

    private enum State
    {
        DEPLOY_JEWEL_ARM,
        DETECT_JEWEL_COLOR,
        DECODE_PICTOGRAPH,
        DISPLACE_JEWEL,
        RETRACT_JEWEL_ARM,
        RESET_JEWEL_ARM,
        DO_DELAY,
        GRAB_LIFT_GLYPH,
        DRIVE_OFF_PLATFORM,
        DRIVE_TO_SAFE_ZONE,
        TURN_TO_CRYPTOBOX,
        ALIGN_CRYPTOBOX,
        MOVE_FORWARD,
        SET_DOWN_GLYPH,
        RELEASE_GLYPH,
        BACK_OFF,
        DONE
    }   //enum State

    private static final String moduleName = "CmdAutoFull";

    private Robot robot;
    private Alliance alliance;
    private double delay;
    private StartPos startPos;
    private DoJewel doJewel;
    private DoCrypto doCrypto;
    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine<State> sm;
    private double targetX = 0.0;
    private double targetY = 0.0;
    private int retryCount = 0;
    private RelicRecoveryVuMark vuMark;

    CmdAutoFull(
            Robot robot, Alliance alliance, double delay, StartPos startPos,
            DoJewel doJewel, DoCrypto doCrypto)
    {
        robot.tracer.traceInfo(
                moduleName, "alliance=%s, delay=%.0f, startPos=%s, doJewel=%s, doCrypto=%s",
                alliance, delay, startPos, doJewel,doCrypto);
        this.robot = robot;
        this.alliance = alliance;
        this.delay = delay;
        this.startPos = startPos;
        this.doJewel = doJewel;
        this.doCrypto = doCrypto;

        //These are used by event system to make the state machine wait for specific events to be triggered
        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);

        sm = new TrcStateMachine<>(moduleName);

        //Set start state here
        sm.start(State.DRIVE_TO_SAFE_ZONE);
        //sm.start(doJewel == FtcAuto.DoJewel.YES? State.DEPLOY_JEWEL_ARM: State.DO_DELAY);
    }   //CmdAutoFull

    //
    // Implements the TrcRobot.RobotCommand interface.
    //

    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        boolean done = !sm.isEnabled();
        //
        // Print debug info.
        //
        State state = sm.getState();
        robot.dashboard.displayPrintf(1, "State: %s", state != null ? sm.getState().toString(): "Disabled");

        //Display other state info here

        if (sm.isReady())
        {
            state = sm.getState();

            switch (state)
            {
                case DEPLOY_JEWEL_ARM:
                    //Enable the jewel color sensor trigger
                    if (robot.jewelColorTrigger != null)
                    {
                        robot.jewelColorTrigger.setEnabled(true);
                    }

                    retryCount = 0;
                    robot.jewelArm.setExtended(true);
                    //Set event to be signaled after specified amount of time
                    timer.set(0.5, event);
                    //State machine will wait for event to be signaled, then move to next state
                    sm.waitForSingleEvent(event, State.RETRACT_JEWEL_ARM);
                    break;
                case DETECT_JEWEL_COLOR:
                    //Disable the jewel color sensor trigger, we're done with it
                    if (robot.jewelColorTrigger != null)
                    {
                        robot.jewelColorTrigger.setEnabled(false);
                    }

                    // determine the jewel color
                    Robot.ObjectColor jewelColor = robot.getObjectColor(robot.jewelColorSensor);

                    robot.tracer.traceInfo(
                            state.toString(), "%d: Color=%s, HSV=[%.0f/%.0f/%.0f]",
                            retryCount, jewelColor.toString(),
                            robot.getObjectHsvHue(robot.jewelColorSensor),
                            robot.getObjectHsvSaturation(robot.jewelColorSensor),
                            robot.getObjectHsvValue(robot.jewelColorSensor));

                    //Stay in this state for up to 10 iterations to detect jewel color
                    if (jewelColor == Robot.ObjectColor.NO && retryCount < 10)
                    {
                        //Incrememnt retry count and exit switch statement. Robot
                        //will remain in current state
                        retryCount++;
                        break;
                    }

                    //If we get here, we either have a color, or we have exceeded our retry count
                    if (jewelColor != Robot.ObjectColor.NO) {
                        //Displace the jewel if we detected a color
                        sm.waitForSingleEvent(event, State.DISPLACE_JEWEL);
                    } else {
                        //retract jewel arm
                        robot.jewelArm.setExtended(false);
                        //proceed to safe zone
                        sm.waitForSingleEvent(event, State.DRIVE_TO_SAFE_ZONE);
                    }
                    break;
                case DISPLACE_JEWEL:
                    //retract jewel arm
                    robot.jewelArm.setExtended(false);

                    //TODO: Displace jewel using timed drive in proper direction

                    //Set event to be signaled after specified amount of time
                    timer.set(0.5, event);
                    //Park in safe zone
                    sm.waitForSingleEvent(event, State.DRIVE_TO_SAFE_ZONE);
                    break;
//                case DECODE_PICTOGRAPH:
//                    vuMark = robot.vuforiaVision.getVuMark();
//                    robot.tracer.traceInfo(state.toString(), "VuMark: %s", vuMark.toString());
//                    if (robot.textToSpeech != null)
//                    {
//                        robot.textToSpeech.speak(
//                                String.format("%s found!", vuMark), TextToSpeech.QUEUE_ADD, null);
//                    }
//
//                    //No wait necessary for decoding, it's synchronous, so just set the next state
//                    sm.setState(State.DISPLACE_JEWEL);
//                    break;
//                case RETRACT_JEWEL_ARM:
//                    robot.jewelArm.setExtended(false);
//                    timer.set(0.3, event);
//                    sm.waitForSingleEvent(event, State.DONE);
//                    break;
//                case RESET_JEWEL_ARM:
//                    robot.jewelArm.setSweepPosition(RobotInfo.JEWEL_ARM_NEUTRAL);
//                    timer.set(0.3, event);
//                    sm.waitForSingleEvent(event, State.DO_DELAY);
//                    break;
                case DRIVE_TO_SAFE_ZONE:
                    //Path depends on start position and alliance
                    SharedFunctions.driveToSafeZone(robot, alliance,startPos,RobotInfo.AUTO_PARKING_POWER);

                    //Set event to be signaled after specified amount of time
                    timer.set(0.5, event);
                    //Park in safe zone
                    sm.waitForSingleEvent(event, State.DONE);
                    break;
//                case DO_DELAY:
//                    //
//                    // Do delay if any.
//                    //
//                    robot.tracer.traceInfo(state.toString(), "Delay=%.0f", delay);
//                    if (delay == 0.0)
//                    {
//                        sm.setState(State.GRAB_LIFT_GLYPH);
//                    }
//                    else
//                    {
//                        timer.set(delay, event);
//                        sm.waitForSingleEvent(event, State.GRAB_LIFT_GLYPH);
//                    }
//                    break;

//                case GRAB_LIFT_GLYPH:
//                    robot.glyphGrabber.setPosition(RobotInfo.GLYPH_GRABBER_CLOSE);
//                    robot.glyphElevator.setPosition(RobotInfo.ELEVATOR_MID_HEIGHT, event, 2.0);
//                    sm.waitForSingleEvent(event, State.DRIVE_OFF_PLATFORM);
//                    break;

//                case DRIVE_OFF_PLATFORM:
//                    robot.encoderYPidCtrl.setOutputRange(-0.5, 0.5);
//                    targetX = 0.0;
//                    targetY = alliance == FtcAuto.Alliance.RED_ALLIANCE ? -22.0 : 26.0;
//                    robot.targetHeading = 0.0;
//
//                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event, 2.0);
//                    sm.waitForSingleEvent(event, State.TURN_TO_CRYPTOBOX);
//                    break;
//
//                case TURN_TO_CRYPTOBOX:
//                    robot.encoderYPidCtrl.setOutputRange(-1.0, 1.0);
//                    targetX = 0.0;
//                    targetY = 0.0;
//                    robot.targetHeading =
//                            startPos == FtcAuto.StartPos.FAR ? -90.0 :
//                                    alliance == FtcAuto.Alliance.RED_ALLIANCE && startPos == FtcAuto.StartPos.NEAR ? 180.0 : 0.0;
//
//                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event, 3.0);
//                    //sm.waitForSingleEvent(event, State.ALIGN_CRYPTOBOX);
//                    sm.waitForSingleEvent(event, State.DONE);
//                    break;

//                case ALIGN_CRYPTOBOX:
//                    if (robot.cryptoColorTrigger != null)
//                    {
//                        robot.redCryptoBarCount = 0;
//                        robot.blueCryptoBarCount = 0;
//                        robot.cryptoColorTrigger.setEnabled(true);
//                    }
//
//                    if (alliance == FtcAuto.Alliance.RED_ALLIANCE)
//                    {
//                        if (startPos == FtcAuto.StartPos.NEAR)
//                        {
//                            if (vuMark == RelicRecoveryVuMark.LEFT)
//                            {
//                                targetX = RED_NEAR_LEFT_COL_OFFSET_IN;
//                            }
//                            else if (vuMark == RelicRecoveryVuMark.RIGHT)
//                            {
//                                targetX = RED_NEAR_RIGHT_COL_OFFSET_IN;
//                            }
//                            else
//                            {
//                                targetX = RED_NEAR_CENTER_COL_OFFSET_IN;
//                            }
//                        }
//                        else
//                        {
//                            if (vuMark == RelicRecoveryVuMark.LEFT)
//                            {
//                                targetX = RED_FAR_LEFT_COL_OFFSET_IN;
//                            }
//                            else if (vuMark == RelicRecoveryVuMark.RIGHT)
//                            {
//                                targetX = RED_FAR_RIGHT_COL_OFFSET_IN;
//                            }
//                            else
//                            {
//                                targetX = RED_FAR_CENTER_COL_OFFSET_IN;
//                            }
//                        }
//                    }
//                    else
//                    {
//                        if (startPos == FtcAuto.StartPos.NEAR)
//                        {
//                            if (vuMark == RelicRecoveryVuMark.LEFT)
//                            {
//                                targetX = BLUE_NEAR_LEFT_COL_OFFSET_IN;
//                            }
//                            else if (vuMark == RelicRecoveryVuMark.RIGHT)
//                            {
//                                targetX = BLUE_NEAR_RIGHT_COL_OFFSET_IN;
//                            }
//                            else
//                            {
//                                targetX = BLUE_NEAR_CENTER_COL_OFFSET_IN;
//                            }
//                        }
//                        else
//                        {
//                            if (vuMark == RelicRecoveryVuMark.LEFT)
//                            {
//                                targetX = BLUE_FAR_LEFT_COL_OFFSET_IN;
//                            }
//                            else if (vuMark == RelicRecoveryVuMark.RIGHT)
//                            {
//                                targetX = BLUE_FAR_RIGHT_COL_OFFSET_IN;
//                            }
//                            else
//                            {
//                                targetX = BLUE_FAR_CENTER_COL_OFFSET_IN;
//                            }
//                        }
//                    }
//                    targetY = 0.0;
//
//                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event, 2.0);
//                    sm.waitForSingleEvent(event, State.MOVE_FORWARD);
//                    break;

//                case MOVE_FORWARD:
//                    if (robot.cryptoColorTrigger != null)
//                    {
//                        robot.cryptoColorTrigger.setEnabled(false);
//                    }
//
//                    // Move forward
//                    targetX = 0.0;
//                    if (alliance == FtcAuto.Alliance.RED_ALLIANCE)
//                    {
//                        targetY = 15.0;
//                    }
//                    else
//                    {
//                        targetY = 8.0;
//                    }
//
//                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event, 1.0);
//                    sm.waitForSingleEvent(event, doCrypto == FtcAuto.DoCrypto.NO? State.DONE: State.SET_DOWN_GLYPH);
//                    break;

//                case SET_DOWN_GLYPH:
//                    // lower the elevator
//                    robot.glyphElevator.setPosition(RobotInfo.ELEVATOR_MIN_HEIGHT, event, 2.0);
//                    sm.waitForSingleEvent(event, State.RELEASE_GLYPH);
//                    break;
//
//                case RELEASE_GLYPH:
//                    robot.glyphGrabber.setPosition(RobotInfo.GLYPH_GRABBER_OPEN);
//                    timer.set(0.5, event);
//                    sm.waitForSingleEvent(event, State.BACK_OFF);
//                    break;

//                case BACK_OFF:
//                    targetX = 0.0;
//                    targetY = -4.5;
//
//                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event, 1.0);
//                    sm.waitForSingleEvent(event, State.DONE);
//                    break;

                case DONE:
                default:
                    // open the glyphgrabber servos
                    //robot.glyphGrabber.setPosition(RobotInfo.GLYPH_GRABBER_OPEN);
                    //
                    // We are done.
                    //
                    done = true;
                    sm.stop();
                    break;
            }
            robot.traceStateInfo(elapsedTime, state.toString(), targetX, targetY, robot.targetHeading);
        }

        if (robot.pidDrive.isActive() && (debugXPid || debugYPid || debugTurnPid))
        {
            robot.tracer.traceInfo("Battery", "Voltage=%5.2fV (%5.2fV)",
                    robot.battery.getVoltage(), robot.battery.getLowestVoltage());

            if (debugXPid && targetX != 0.0)
            {
                robot.encoderXPidCtrl.printPidInfo(robot.tracer);
            }

            if (debugYPid && targetY != 0.0)
            {
                robot.encoderYPidCtrl.printPidInfo(robot.tracer);
            }

            if (debugTurnPid)
            {
                robot.gyroPidCtrl.printPidInfo(robot.tracer);
            }
        }

        return done;
    }   //cmdPeriodic

}   //class CmdAutoFull