/*   MIT License
 *   Copyright (c) [2025] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.LayerHeight;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.Artboard;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations.AnimationType;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations.PoliceLights;

import java.util.concurrent.TimeUnit;

/*
 * This code creates a "Configurator" UI which exposes a somewhat limited amount of the functionality
 * available to create animations and artboards to users without needing to work with very much code.
 *
 * This file is not meant to serve as example code - unless you're trying to create a telemetry-based
 * UI for the drivers station.
 * For example code that your team can leverage in your autonomous/teleop code to recall artboards
 * check out the GoBildaPrismArtboardExample. Run this code on your robot and use it to create
 * the artboards you'd like to be able to switch between. This means that switching between animations
 * can be very fast and easy, and the work of creating animations (and sending this information over
 * I²C) only has to happen once!
 *
 */

@TeleOp(name="Prism Configurator", group="Linear OpMode")
//@Disabled

public class GoBildaPrismConfigurator extends LinearOpMode {

    GoBildaPrismDriver prism;

    // an enum which is used in a mini state machine to allow the user to set different colors.
    public enum AnimationColor{
        PRIMARY_COLOR,
        SECONDARY_COLOR,
        TERTIARY_COLOR,
    }
    AnimationColor animationColor = AnimationColor.PRIMARY_COLOR;

    // Set a default style for the Police Lights Animation.
    PoliceLights.PoliceLightsStyle policeLightsStyle = PoliceLights.PoliceLightsStyle.Style1;

    int startPoint = 0; // the start LED for any configured animation
    int endPoint = 12; // the end LED for a configured animation
    int brightness = 50; // the brightness of configured animation
    int period = 1000; // the period of a configured animation
    float speed = 0.5F; // the speed of a configured animation

    int layerSelector = 0; // an integer used to create a cursor to select a layer
    int animationSelector = 1; // animation cursor
    int artboardSelector = 0; // artboard cursor

    String hsbTelemetry; // string meant to send over telemetry that gets updated by hsbViaJoystick()
    String hueTelemetry; // updated by hueViaJoystick()

    /*
     * An array of colors passed to the SingleFill animation.
     */
    Color[] singleFillColors = {
        Color.RED, Color.WHITE, Color.BLUE
    };

    AnimationType selectedAnimation = AnimationType.SOLID; // store the animation that is being selected.

    /*
     * the enum powering the main state machine that this code moves through, each state represents
     * a page the user can see displayed via telemetry on the driver's station.
     */
    public enum ConfigState{
        WELCOME_SCREEN,
        SELECT_LAYER,
        SET_ENDPOINTS,
        SELECT_ANIMATION,
        CONFIGURE_ANIMATION,
        SET_BRIGHTNESS,
        SET_SPEED,
        FORK_IN_THE_ROAD,
        SAVE_TO_ARTBOARD,
        COMPLETE;
    }

    ConfigState configState = ConfigState.WELCOME_SCREEN;

    /*
     * This is actually a bit of a duplicate of the LayerHeight found in PrismAnimations. This adds
     * an animation slot which can be stored at each position in the enum. This is not required for
     * the Prism side, but I want to be able to show a user what animation is stored at what layer
     * after they've created their first animation.
     */
    public enum Layers{
        LAYER_0 (AnimationType.NONE,0,LayerHeight.LAYER_0),
        LAYER_1 (AnimationType.NONE,1,LayerHeight.LAYER_1),
        LAYER_2 (AnimationType.NONE,2,LayerHeight.LAYER_2),
        LAYER_3 (AnimationType.NONE,3,LayerHeight.LAYER_3),
        LAYER_4 (AnimationType.NONE,4,LayerHeight.LAYER_4),
        LAYER_5 (AnimationType.NONE,5,LayerHeight.LAYER_5),
        LAYER_6 (AnimationType.NONE,6,LayerHeight.LAYER_6),
        LAYER_7 (AnimationType.NONE,7,LayerHeight.LAYER_7),
        LAYER_8 (AnimationType.NONE,8,LayerHeight.LAYER_8),
        LAYER_9 (AnimationType.NONE,9,LayerHeight.LAYER_9);

        private AnimationType animationType;
        private final int index;
        private final LayerHeight layerHeight;

        Layers(AnimationType animationType, int index, LayerHeight layerHeight){
            this.animationType = animationType;
            this.index = index;
            this.layerHeight = layerHeight;
        }
    }

    /*
     * This enum captures the kind of speed we can control on the animation.
     */
    public enum SpeedType{
        NO_SPEED,
        PERIOD_ONLY,
        SPEED_ONLY,
        PERIOD_AND_SPEED,
    }

    Layers selectedLayer = Layers.LAYER_0;

    Artboard selectedArtboard = Artboard.ARTBOARD_0;

    /*
     * Create each Prism Animation which can be customized by the user.
     */
    PrismAnimations.Solid solid = new PrismAnimations.Solid();
    PrismAnimations.Solid endpointsAnimation = new PrismAnimations.Solid();
    PrismAnimations.Blink blink = new PrismAnimations.Blink();
    PrismAnimations.Pulse pulse = new PrismAnimations.Pulse();
    PrismAnimations.SineWave sineWave = new PrismAnimations.SineWave();
    PrismAnimations.DroidScan droidScan = new PrismAnimations.DroidScan();
    PrismAnimations.Rainbow rainbow = new PrismAnimations.Rainbow();
    PrismAnimations.Snakes snakes = new PrismAnimations.Snakes();
    PrismAnimations.Random random = new PrismAnimations.Random();
    PrismAnimations.Sparkle sparkle = new PrismAnimations.Sparkle();
    PrismAnimations.SingleFill singleFill = new PrismAnimations.SingleFill();
    PrismAnimations.RainbowSnakes rainbowSnakes = new PrismAnimations.RainbowSnakes();
    PoliceLights policeLights = new PoliceLights();


    @Override
    public void runOpMode() {
        prism = hardwareMap.get(GoBildaPrismDriver.class,"prism");

        telemetry.addLine("Welcome to the Prism Configurator, enjoy these fun stats " +
                "and click the 'Play' button to continue");
        telemetry.addLine("");

        telemetry.addData("Device ID", prism.getDeviceID());
        telemetry.addLine(prism.getFirmwareVersionString());
        telemetry.addLine(prism.getHardwareVersionString());
        telemetry.addData("Power Cycle Count", prism.getPowerCycleCount());
        telemetry.addData("Run Time (Minutes)",prism.getRunTime(TimeUnit.MINUTES));
        telemetry.addData("Run Time (Hours)",prism.getRunTime(TimeUnit.HOURS));
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            switch (configState){
                case WELCOME_SCREEN:
                    welcomeScreen();
                    if(gamepad1.aWasPressed()){
                        //if the user presses "a" we move onto the next state.
                        prism.clearAllAnimations();
                        configState = ConfigState.SELECT_LAYER;
                    }
                    break;
                case SELECT_LAYER:
                    /*
                     * You can't convince me that this is the correct way to implement a cursor,
                     * but you also can't tell me this one doesn't work.
                     * We allow the user to increment the layerSelecter variable, before constraining
                     * it to within a valid range. We then pass that into our "selectLayer()"
                     * function which displays the UI elements via telemetry and saves the selected
                     * layer every loop.
                     */
                    if(gamepad1.dpadUpWasPressed()){
                        layerSelector -=1;
                    }
                    if(gamepad1.dpadDownWasPressed()){
                        layerSelector +=1;
                    }
                    layerSelector = Math.min(9, Math.max(0,layerSelector));
                    selectLayer(layerSelector);
                    if(gamepad1.aWasPressed()){
                        configureEndpointsAnimation(true);
                        configState = ConfigState.SET_ENDPOINTS;
                    }
                    break;
                case SET_ENDPOINTS:
                    configureEndPoints();
                    if(gamepad1.aWasPressed()){
                        telemetry.clear();
                        selectAnimation(animationSelector);
                        configState = ConfigState.SELECT_ANIMATION;
                    }
                    if(gamepad1.bWasPressed()){
                        configState =  ConfigState.SELECT_LAYER;
                    }
                    break;
                case SELECT_ANIMATION:
                    if(gamepad1.dpadUpWasPressed()){
                        animationSelector -=1;
                        animationSelector = Math.min(12, Math.max(1,animationSelector));
                        selectAnimation(animationSelector);
                    }
                    if(gamepad1.dpadDownWasPressed()){
                        animationSelector +=1;
                        animationSelector = Math.min(12, Math.max(1,animationSelector));
                        selectAnimation(animationSelector);
                    }
                    if(gamepad1.aWasPressed()){
                        selectedLayer.animationType = selectedAnimation;
                        configState = ConfigState.CONFIGURE_ANIMATION;
                    }
                    if(gamepad1.bWasPressed()){
                        configureEndpointsAnimation(true);
                        configState =  ConfigState.SET_ENDPOINTS;
                    }
                    break;
                case CONFIGURE_ANIMATION:
                    configureAnimation(true,false);
                    if(gamepad1.aWasPressed()){
                        configState = ConfigState.SET_BRIGHTNESS;
                    }
                    if(gamepad1.bWasPressed()){
                        telemetry.clear();
                        selectAnimation(animationSelector);
                        configState =  ConfigState.SELECT_ANIMATION;
                    }
                    break;
                case SET_BRIGHTNESS:
                    configureBrightness();
                    if(gamepad1.aWasPressed()){
                        configState = ConfigState.SET_SPEED;
                    }
                    if(gamepad1.bWasPressed()){
                        configState =  ConfigState.CONFIGURE_ANIMATION;
                    }
                    break;
                case SET_SPEED:
                    if(configureSpeed()){
                        configState = ConfigState.FORK_IN_THE_ROAD;
                    }
                    if(gamepad1.aWasPressed()){
                        configState = ConfigState.FORK_IN_THE_ROAD;
                    }
                    if(gamepad1.bWasPressed()){
                        configState = ConfigState.SET_BRIGHTNESS;
                    }
                    break;
                case FORK_IN_THE_ROAD:
                    forkInTheRoad();
                    if(gamepad1.aWasPressed()){
                        telemetry.clear();
                        configState =  ConfigState.SAVE_TO_ARTBOARD;
                        selectArtboard(artboardSelector);
                    }
                    if(gamepad1.bWasPressed()){
                        if(configureSpeed()){
                            configState = ConfigState.SET_BRIGHTNESS;
                        } else configState = ConfigState.SET_SPEED;
                    }
                    if(gamepad1.yWasPressed()){
                        configState = ConfigState.SELECT_LAYER;
                    }
                    if(gamepad1.xWasPressed()){
                        configState = ConfigState.WELCOME_SCREEN;
                    }
                    break;
                case SAVE_TO_ARTBOARD:
                    if(gamepad1.dpadUpWasPressed()){
                        artboardSelector -=1;
                        artboardSelector = Math.min(7, Math.max(0,artboardSelector));
                        selectArtboard(artboardSelector);
                    }
                    if(gamepad1.dpadDownWasPressed()){
                        artboardSelector +=1;
                        artboardSelector = Math.min(7, Math.max(0,artboardSelector));
                        selectArtboard(artboardSelector);
                    }
                    if(gamepad1.aWasPressed()){
                        prism.saveCurrentAnimationsToArtboard(selectedArtboard);
                        prism.enableDefaultBootArtboard(true);
                        configState = ConfigState.COMPLETE;
                    }
                    if(gamepad1.bWasPressed()){
                        configState = ConfigState.FORK_IN_THE_ROAD;
                    }
                    break;
                case COMPLETE:
                    telemetry.addLine("Artboard saved!");
                    telemetry.addLine("");
                    telemetry.addLine("Press X to return to the beginning.");
                    if(gamepad1.xWasPressed()){
                        configState = ConfigState.WELCOME_SCREEN;
                    }
                    break;
            }
            telemetry.update();
            sleep(20);
        }
    }

    /*
     * through this code I try and keep as much code as possible inside of functions, with how
     * complex our main state machine is, stuff like telemetry is is nice to keep only where we need it.
     */
    public void welcomeScreen(){
        telemetry.addLine("Welcome to the goBILDA Prism Configurator!");
        telemetry.addLine("Hold on tight - we were cooking when we made this.");
        telemetry.addLine("");
        telemetry.addLine("Core to understanding how to use this product is knowing these three terms:");
        telemetry.addLine("Animations: (Like RAINBOW or BLINK) - These have properties you can configure, " +
                "like their color. And they can have unique start and end points.");
        telemetry.addLine("");
        telemetry.addLine("Layers: There are 10 layers, each of which can store an animation. " +
                "These are hierarchical. So an Animation on layer 5 will cover an animation on layer 2 " +
                "if they overlap. You can use start and end points to have layers overlap to create new patterns!" +
                "Or show multiple animations at once on different LEDs.");
        telemetry.addLine("");
        telemetry.addLine("Artboards: An Artboard is a set of 10 layers which is stored on the Prism. " +
                "you can have up to 8 unique Artboards. Artboards are easy and computationally fast to switch between. " +
                "We recommend that you configure your Artboards for your robot once and then switch between them.");
        telemetry.addLine("");
        telemetry.addLine("Press A to continue");
        resetStoredAnimations();
    }

    public void selectLayer(int selector){
        telemetry.addLine("Select the Layer that you wish to save an Animation to.");
        telemetry.addLine("Use D-Pad up and D-Pad down to navigate through the layers.");
        telemetry.addLine("");
        telemetry.addData("LAYER_0 Animation", Layers.LAYER_0.animationType+layerCursor(Layers.LAYER_0,selector));
        telemetry.addData("LAYER_1 Animation", Layers.LAYER_1.animationType+layerCursor(Layers.LAYER_1,selector));
        telemetry.addData("LAYER_2 Animation", Layers.LAYER_2.animationType+layerCursor(Layers.LAYER_2,selector));
        telemetry.addData("LAYER_3 Animation", Layers.LAYER_3.animationType+layerCursor(Layers.LAYER_3,selector));
        telemetry.addData("LAYER_4 Animation", Layers.LAYER_4.animationType+layerCursor(Layers.LAYER_4,selector));
        telemetry.addData("LAYER_5 Animation", Layers.LAYER_5.animationType+layerCursor(Layers.LAYER_5,selector));
        telemetry.addData("LAYER_6 Animation", Layers.LAYER_6.animationType+layerCursor(Layers.LAYER_6,selector));
        telemetry.addData("LAYER_7 Animation", Layers.LAYER_7.animationType+layerCursor(Layers.LAYER_7,selector));
        telemetry.addData("LAYER_8 Animation", Layers.LAYER_8.animationType+layerCursor(Layers.LAYER_8,selector));
        telemetry.addData("LAYER_9 Animation", Layers.LAYER_9.animationType+layerCursor(Layers.LAYER_9,selector));
        telemetry.addLine("");
        telemetry.addLine("Press A to continue");
    }

    public String layerCursor(Layers layer, int selector){
        if(layer.index == selector){
            selectedLayer = layer;
            return "<--";
        }else return "";
    }

    public void selectAnimation(int animationSelector){
        telemetry.addLine("Select the Animation that you wish to use");
        telemetry.addLine("Use D-Pad up and D-Pad down to navigate through the Animations.");
        telemetry.addLine("");
        telemetry.addData("Solid",animationCursor(AnimationType.SOLID,animationSelector));
        telemetry.addData("Blink",animationCursor(AnimationType.BLINK,animationSelector));
        telemetry.addData("Pulse",animationCursor(AnimationType.PULSE,animationSelector));
        telemetry.addData("Sine Wave",animationCursor(AnimationType.SINE_WAVE,animationSelector));
        telemetry.addData("Droid Scan",animationCursor(AnimationType.DROID_SCAN,animationSelector));
        telemetry.addData("Rainbow",animationCursor(AnimationType.RAINBOW,animationSelector));
        telemetry.addData("Snakes",animationCursor(AnimationType.SNAKES,animationSelector));
        telemetry.addData("Random",animationCursor(AnimationType.RANDOM,animationSelector));
        telemetry.addData("Sparkle",animationCursor(AnimationType.SPARKLE,animationSelector));
        telemetry.addData("Single Fill",animationCursor(AnimationType.SINGLE_FILL,animationSelector));
        telemetry.addData("Rainbow Snakes",animationCursor(AnimationType.RAINBOW_SNAKES,animationSelector));
        telemetry.addData("Police Lights",animationCursor(AnimationType.POLICE_LIGHTS,animationSelector));
        telemetry.addLine("");
        telemetry.addLine("Press A to continue");
        telemetry.addLine("Press B to go back");
    }

    public String animationCursor(AnimationType animationType, int selector){
        if(animationType.AnimationTypeIndex == selector){
            selectedAnimation = animationType;
            configureAnimation(false,true);
            return "<--";
        }else return "";
    }

    public void configureEndPoints(){
        if(gamepad1.dpadLeftWasPressed()){
            startPoint -= 1;
            startPoint = Math.min(Math.min(255,Math.max(0,startPoint)),endPoint-1);
            configureEndpointsAnimation(false);
        }
        if(gamepad1.dpadRightWasPressed()){
            startPoint += 1;
            startPoint = Math.min(Math.min(255,Math.max(0,startPoint)),endPoint-1);
            configureEndpointsAnimation(false);
        }
        if(gamepad1.leftBumperWasPressed()){
            endPoint -= 1;
            endPoint = Math.max(Math.min(255,Math.max(0,endPoint)),startPoint+1);
            configureEndpointsAnimation(false);
        }
        if(gamepad1.rightBumperWasPressed()){
            endPoint += 1;
            endPoint = Math.max(Math.min(255,Math.max(0,endPoint)),startPoint+1);
            configureEndpointsAnimation(false);
        }

        telemetry.addLine("Set the start and stop point for each LED");
        telemetry.addLine("");
        telemetry.addLine("Use the d-pad to set the start point, d-pad right moves it further from the Prism. " +
                "d-pad left moves it closer.");
        telemetry.addLine("Bumpers move the endpoint, The left bumper moves it closer to the Prism, right moves it further.");
        telemetry.addData("Start Point", startPoint);
        telemetry.addData("End Point", endPoint);
        telemetry.addLine("");
        telemetry.addLine("Press A to Continue");
        telemetry.addLine("Press B to go back");
    }

    public void configureBrightness(){
        if(gamepad1.dpadDownWasPressed()){
            brightness -= 10;
            brightness = Math.min(100,Math.max(0,brightness));
            configureAnimation(false,false);
        }
        if(gamepad1.dpadUpWasPressed()){
            brightness += 10;
            brightness = Math.min(100,Math.max(0,brightness));
            configureAnimation(false,false);
        }

        telemetry.addLine("Set the brightness for this animation");
        telemetry.addLine("");
        telemetry.addLine("Use the d-pad to adjust the brightness, up increases, down decreases.");
        telemetry.addLine("");
        telemetry.addData("Brightness", brightness);
        telemetry.addLine("");
        telemetry.addLine("Press A to Continue");
        telemetry.addLine("Press B to go back");
    }

    /**
     * Returns: True if we should skip this animation type.
     */
    public boolean configureSpeed(){
        switch (speedFromAnimation(selectedAnimation)){
            case NO_SPEED: return true;
            case SPEED_ONLY:
                if(gamepad1.dpadUpWasPressed()){
                    speed += 0.1f;
                    speed = Math.min(1.0f, Math.max(0,speed));
                    configureAnimation(false,false);
                }
                if(gamepad1.dpadDownWasPressed()){
                    speed -= 0.1f;
                    speed = Math.min(1.0f, Math.max(0,speed));
                    configureAnimation(false,false);
                }
                telemetry.addLine("Set the speed for this animation from 0 to 1");
                telemetry.addLine("");
                telemetry.addLine("Use the d-pad to adjust the speed, up increases, down decreases.");
                telemetry.addLine("");
                telemetry.addData("Speed", speed);
                telemetry.addLine("");
                telemetry.addLine("Press A to Continue");
                telemetry.addLine("Press B to go back");
            break;
            case PERIOD_ONLY:
                if(gamepad1.dpadUpWasPressed()){
                    if(period < 401){
                        period += 100;
                    } else period += 500;
                    period = Math.min(300000, Math.max(0,period));
                    configureAnimation(false,false);
                }
                if(gamepad1.dpadDownWasPressed()){
                    if(period < 501){
                        period -= 100;
                    } else period -= 500;
                    period = Math.min(300000, Math.max(0,period));
                    configureAnimation(false,false);
                }
                telemetry.addLine("Set the period for this animation in Milliseconds");
                telemetry.addLine("");
                telemetry.addLine("Use the d-pad to adjust the period, up increases, down decreases.");
                telemetry.addLine("");
                telemetry.addData("Period", period);
                telemetry.addLine("");
                telemetry.addLine("Press A to Continue");
                telemetry.addLine("Press B to go back");
            break;
            case PERIOD_AND_SPEED:
                if(gamepad1.dpadUpWasPressed()){
                    if(period < 401){
                        period += 100;
                    } else period += 500;
                    period = Math.min(300000, Math.max(0,period));
                    configureAnimation(false,false);
                }
                if(gamepad1.dpadDownWasPressed()){
                    if(period < 501){
                        period -= 100;
                    } else period -= 500;
                    period = Math.min(300000, Math.max(0,period));
                    configureAnimation(false,false);
                }
                if(gamepad1.dpadRightWasPressed()){
                    speed += 0.1f;
                    speed = Math.min(1.0f, Math.max(0,speed));
                    configureAnimation(false,false);
                }
                if(gamepad1.dpadLeftWasPressed()){
                    speed -= 0.1f;
                    speed = Math.min(1.0f, Math.max(0,speed));
                    configureAnimation(false,false);
                }
                telemetry.addLine("Set the period for this animation in Milliseconds");
                telemetry.addLine("");
                telemetry.addLine("Set the speed for this animation from 0-1");
                telemetry.addLine("");
                telemetry.addLine("Use the d-pad to adjust the period, up increases, down decreases.");
                telemetry.addLine("");
                telemetry.addData("Period", period);
                telemetry.addLine("");
                telemetry.addLine("Use the d-pad to adjust the speed, right increases, left decreases.");
                telemetry.addLine("");
                telemetry.addData("Speed", speed);
                telemetry.addLine("");
                telemetry.addLine("Press A to Continue");
                telemetry.addLine("Press B to go back");
            break;
        }
        return false;
    }

    public SpeedType speedFromAnimation(AnimationType animationType){
        if(animationType == AnimationType.BLINK || animationType == AnimationType.PULSE ||
                animationType == AnimationType.SPARKLE || animationType == AnimationType.POLICE_LIGHTS){
            return SpeedType.PERIOD_ONLY;
        }
        if (animationType == AnimationType.DROID_SCAN || animationType == AnimationType.RAINBOW ||
                animationType == AnimationType.SNAKES || animationType == AnimationType.RANDOM ||
                animationType == AnimationType.RAINBOW_SNAKES){
            return SpeedType.SPEED_ONLY;
        }
        if(animationType == AnimationType.SINE_WAVE || animationType == AnimationType.SINGLE_FILL){
            return SpeedType.PERIOD_AND_SPEED;
        }
        else return SpeedType.NO_SPEED;
    }

    public void forkInTheRoad(){
        telemetry.addLine("If you are done with your creation, press A to save it to an Artboard.");
        telemetry.addLine("");
        telemetry.addLine("To go back, press B.");
        telemetry.addLine("");
        telemetry.addLine("If you'd instead like to layer another animation on top of this " +
                "one, press Y.");
        telemetry.addLine("");
        telemetry.addLine("Press X to return to the start and clear currently set animations.");
    }

    public void selectArtboard(int artboardSelector){
        telemetry.addLine("Select the Artboard that you wish to save to");
        telemetry.addLine("Use D-Pad up and D-Pad down to navigate through the Artboards.");
        telemetry.addLine("");
        telemetry.addData("Artboard 0",artboardCursor(Artboard.ARTBOARD_0,artboardSelector));
        telemetry.addData("Artboard 1",artboardCursor(Artboard.ARTBOARD_1,artboardSelector));
        telemetry.addData("Artboard 2",artboardCursor(Artboard.ARTBOARD_2,artboardSelector));
        telemetry.addData("Artboard 3",artboardCursor(Artboard.ARTBOARD_3,artboardSelector));
        telemetry.addData("Artboard 4",artboardCursor(Artboard.ARTBOARD_4,artboardSelector));
        telemetry.addData("Artboard 5",artboardCursor(Artboard.ARTBOARD_5,artboardSelector));
        telemetry.addData("Artboard 6",artboardCursor(Artboard.ARTBOARD_6,artboardSelector));
        telemetry.addData("Artboard 7",artboardCursor(Artboard.ARTBOARD_7,artboardSelector));
        telemetry.addLine("");
        telemetry.addLine("Press A to save");
        telemetry.addLine("Press B to go back");
    }

    public String artboardCursor(Artboard artboard, int selector){
        if(artboard.index == selector){
            selectedArtboard = artboard;
            return "<--";
        }else return "";
    }

    public void configureAnimation(boolean showTelemetry, boolean isBeingInserted){
        switch(selectedAnimation){
            case SOLID:
                configureSolid(showTelemetry, isBeingInserted);
                break;
            case BLINK:
                configureBlink(showTelemetry, isBeingInserted);
                break;
            case PULSE:
                configurePulse(showTelemetry, isBeingInserted);
                break;
            case SINE_WAVE:
                configureSineWave(showTelemetry, isBeingInserted);
                break;
            case DROID_SCAN:
                configureDroidScan(showTelemetry, isBeingInserted);
                break;
            case RAINBOW:
                configureRainbow(showTelemetry, isBeingInserted);
                break;
            case SNAKES:
                configureSnakes(showTelemetry, isBeingInserted);
                break;
            case RANDOM:
                configureRandom(showTelemetry, isBeingInserted);
                break;
            case SPARKLE:
                configureSparkle(showTelemetry, isBeingInserted);
                break;
            case SINGLE_FILL:
                configureSingleFill(showTelemetry, isBeingInserted);
                break;
            case RAINBOW_SNAKES:
                configureRainbowSnakes(showTelemetry, isBeingInserted);
                break;
            case POLICE_LIGHTS:
                configurePoliceLights(showTelemetry, isBeingInserted);
                break;
        }
    }

    public void configureEndpointsAnimation(boolean isBeingInserted){
        endpointsAnimation.setStartIndex(startPoint);
        endpointsAnimation.setStopIndex(endPoint);
        if(isBeingInserted){
            prism.insertAndUpdateAnimation(selectedLayer.layerHeight, endpointsAnimation);
        } else {
            prism.updateAnimationFromIndex(selectedLayer.layerHeight);
        }
    }

    /*
     * Past this point, we have a function to create each type of animation and configure the
     * specifics to it. As often as I could, you'll find that actions that multiple animations
     * share (like setPrimaryColor) get abstracted into their own functions (here we have
     * "hsbViaJoystick) to do the work. This avoids copy and pasting code.
     */

    public void configureSolid(boolean showTelemetry, boolean isBeingInserted){
        solid.setPrimaryColor(hsbViaJoystick(solid.getPrimaryColor()));
        solid.setStartIndex(startPoint);
        solid.setStopIndex(endPoint);
        solid.setBrightness(brightness);

        if(isBeingInserted){
            prism.insertAndUpdateAnimation(selectedLayer.layerHeight, solid);
        } else {
            prism.updateAnimationFromIndex(selectedLayer.layerHeight);
        }

        if(showTelemetry){
            telemetry.addLine("Selected Animation: Solid");
            showHsbTelemetry();
            telemetry.addLine(hsbTelemetry);
            telemetry.addLine("Press A to Continue");
            telemetry.addLine("Press B to go back");
        }
    }

    public void configureBlink(boolean showTelemetry, boolean isBeingInserted){
        switch (animationColor){
            case PRIMARY_COLOR:
                blink.setPrimaryColor(hsbViaJoystick(blink.getPrimaryColor()));
                break;
            case SECONDARY_COLOR:
                blink.setSecondaryColor(hsbViaJoystick(blink.getSecondaryColor()));
                break;
        }
        toggleThroughColors(gamepad1.yWasPressed(),false);

        blink.setStartIndex(startPoint);
        blink.setStopIndex(endPoint);
        blink.setBrightness(brightness);
        blink.setPeriod(period);
        blink.setPrimaryColorPeriod(period/2);

        if(isBeingInserted){
            prism.insertAndUpdateAnimation(selectedLayer.layerHeight, blink);
        } else {
            prism.updateAnimationFromIndex(selectedLayer.layerHeight);
        }

        if(showTelemetry){
            telemetry.addLine("Selected Animation: Blink");
            showHsbTelemetry();
            telemetry.addLine(hsbTelemetry);
            telemetry.addLine("");
            telemetry.addLine("Click the Y button to switch between setting the primary and secondary color");
            telemetry.addLine(animationColor.toString());
            telemetry.addLine("");
            telemetry.addLine("Press A to Continue");
            telemetry.addLine("Press B to go back");
        }
    }

    public void configurePulse(boolean showTelemetry, boolean isBeingInserted){
        switch (animationColor){
            case PRIMARY_COLOR:
                pulse.setPrimaryColor(hsbViaJoystick(pulse.getPrimaryColor()));
                break;
            case SECONDARY_COLOR:
                pulse.setSecondaryColor(hsbViaJoystick(pulse.getSecondaryColor()));
                break;
        }

        toggleThroughColors(gamepad1.yWasPressed(),false);

        pulse.setStartIndex(startPoint);
        pulse.setStopIndex(endPoint);
        pulse.setBrightness(brightness);
        pulse.setPeriod(period);

        if(isBeingInserted){
            prism.insertAndUpdateAnimation(selectedLayer.layerHeight, pulse);
        } else {
            prism.updateAnimationFromIndex(selectedLayer.layerHeight);
        }

        if(showTelemetry){
            telemetry.addLine("Selected Animation: Pulse");
            showHsbTelemetry();
            telemetry.addLine(hsbTelemetry);
            telemetry.addLine("");
            telemetry.addLine("Click the Y button to switch between setting the primary and secondary color");
            telemetry.addLine(animationColor.toString());
            telemetry.addLine("");
            telemetry.addLine("Press A to Continue");
            telemetry.addLine("Press B to go back");
        }
    }

    public void configureSineWave(boolean showTelemetry, boolean isBeingInserted){
        switch (animationColor){
            case PRIMARY_COLOR:
                sineWave.setPrimaryColor(hsbViaJoystick(sineWave.getPrimaryColor()));
                break;
            case SECONDARY_COLOR:
                sineWave.setSecondaryColor(hsbViaJoystick(sineWave.getSecondaryColor()));
                break;
        }

        toggleThroughColors(gamepad1.yWasPressed(),false);

        sineWave.setStartIndex(startPoint);
        sineWave.setStopIndex(endPoint);
        sineWave.setBrightness(brightness);
        sineWave.setPeriod(period);
        sineWave.setSpeed(speed);

        if(isBeingInserted){
            prism.insertAndUpdateAnimation(selectedLayer.layerHeight, sineWave);
        } else {
            prism.updateAnimationFromIndex(selectedLayer.layerHeight);
        }

        if(showTelemetry){
            telemetry.addLine("Selected Animation: Sine Wave");
            showHsbTelemetry();
            telemetry.addLine(hsbTelemetry);
            telemetry.addLine("");
            telemetry.addLine("Click the Y button to switch between setting the primary and secondary color");
            telemetry.addLine(animationColor.toString());
            telemetry.addLine("");
            telemetry.addLine("Press A to Continue");
            telemetry.addLine("Press B to go back");
        }
    }

    public void configureDroidScan(boolean showTelemetry, boolean isBeingInserted){
        switch (animationColor){
            case PRIMARY_COLOR:
                droidScan.setPrimaryColor(hsbViaJoystick(droidScan.getPrimaryColor()));
                break;
            case SECONDARY_COLOR:
                droidScan.setSecondaryColor(hsbViaJoystick(droidScan.getSecondaryColor()));
                break;
        }

        toggleThroughColors(gamepad1.yWasPressed(),false);

        droidScan.setStartIndex(startPoint);
        droidScan.setStopIndex(endPoint);
        droidScan.setBrightness(brightness);
        droidScan.setSpeed(speed);

        if(isBeingInserted){
            prism.insertAndUpdateAnimation(selectedLayer.layerHeight, droidScan);
        } else {
            prism.updateAnimationFromIndex(selectedLayer.layerHeight);
        }

        if(showTelemetry){
            telemetry.addLine("Selected Animation: Droid Scan");
            showHsbTelemetry();
            telemetry.addLine(hsbTelemetry);
            telemetry.addLine("");
            telemetry.addLine("Click the Y button to switch between setting the primary and secondary color");
            telemetry.addLine(animationColor.toString());
            telemetry.addLine("");
            telemetry.addLine("Press A to Continue");
            telemetry.addLine("Press B to go back");
        }
    }

    public void configureRainbow(boolean showTelemetry, boolean isBeingInserted){
        float[] hues = hueViaJoystick(rainbow.getStartHue(),rainbow.getStopHue());
        rainbow.setStartHue(hues[0]);
        rainbow.setStopHue(hues[1]);

        rainbow.setStartIndex(startPoint);
        rainbow.setStopIndex(endPoint);
        rainbow.setBrightness(brightness);
        rainbow.setSpeed(speed);

        if(isBeingInserted){
            prism.insertAndUpdateAnimation(selectedLayer.layerHeight, rainbow);
        } else {
            prism.updateAnimationFromIndex(selectedLayer.layerHeight);
        }
        if(showTelemetry){
            telemetry.addLine("Selected Animation: Rainbow");
            telemetry.addLine("Move the joysticks to configure the range of the rainbow");
            telemetry.addLine("Left Joystick X (side-to-side) changes the start hue");
            telemetry.addLine("Right Joystick x (side-to-side) changes the end hue");
            telemetry.addLine("");
            telemetry.addLine(hueTelemetry);
            telemetry.addLine("");
            telemetry.addLine("Press A to Continue");
            telemetry.addLine("Press B to go back");
        }
    }

    public void configureSnakes(boolean showTelemetry, boolean isBeingInserted){
        snakes.setColors(hsbViaJoystick(snakes.getColors()[0]));

        snakes.setStartIndex(startPoint);
        snakes.setStopIndex(endPoint);
        snakes.setBrightness(brightness);
        snakes.setSpeed(speed);

        if(isBeingInserted){
            prism.insertAndUpdateAnimation(selectedLayer.layerHeight, snakes);
        } else {
            prism.updateAnimationFromIndex(selectedLayer.layerHeight);
        }

        if(showTelemetry) {
            telemetry.addLine("Selected Animation: Snakes");
            showHsbTelemetry();
            telemetry.addLine(hsbTelemetry);
            telemetry.addLine("");
            telemetry.addLine("Press A to Continue");
            telemetry.addLine("Press B to go back");
        }
    }

    public void configureRandom(boolean showTelemetry, boolean isBeingInserted){
        float[] hues = hueViaJoystick(random.getStartHue(), random.getStopHue());
        random.setStartHue(hues[0]);
        random.setStopHue(hues[1]);

        random.setStartIndex(startPoint);
        random.setStopIndex(endPoint);
        random.setBrightness(brightness);
        random.setSpeed(speed);

        if(isBeingInserted){
            prism.insertAndUpdateAnimation(selectedLayer.layerHeight, random);
        } else {
            prism.updateAnimationFromIndex(selectedLayer.layerHeight);
        }

        if(showTelemetry){
            telemetry.addLine("Selected Animation: Random");
            telemetry.addLine("Move the joysticks to configure the range of the random colors");
            telemetry.addLine("Left Joystick X (side-to-side) changes the start hue");
            telemetry.addLine("Right Joystick x (side-to-side) changes the end hue");
            telemetry.addLine("");
            telemetry.addLine(hueTelemetry);
            telemetry.addLine("");
            telemetry.addLine("Press A to Continue");
            telemetry.addLine("Press B to go back");
        }
    }

    public void configureSparkle(boolean showTelemetry, boolean isBeingInserted){
        switch (animationColor){
            case PRIMARY_COLOR:
                sparkle.setPrimaryColor(hsbViaJoystick(sparkle.getPrimaryColor()));
                break;
            case SECONDARY_COLOR:
                sparkle.setSecondaryColor(hsbViaJoystick(sparkle.getSecondaryColor()));
                break;
        }

        toggleThroughColors(gamepad1.yWasPressed(),false);

        sparkle.setStartIndex(startPoint);
        sparkle.setStopIndex(endPoint);
        sparkle.setBrightness(brightness);
        sparkle.setPeriod(period);

        if(isBeingInserted){
            prism.insertAndUpdateAnimation(selectedLayer.layerHeight, sparkle);
        } else {
            prism.updateAnimationFromIndex(selectedLayer.layerHeight);
        }

        if(showTelemetry){
            telemetry.addLine("Selected Animation: Sparkle");
            showHsbTelemetry();
            telemetry.addLine(hsbTelemetry);
            telemetry.addLine("");
            telemetry.addLine("Click the Y button to switch between setting the primary and secondary color");
            telemetry.addLine(animationColor.toString());
            telemetry.addLine("");
            telemetry.addLine("Press A to Continue");
            telemetry.addLine("Press B to go back");
        }
    }

    public void configureSingleFill(boolean showTelemetry, boolean isBeingInserted){
        switch (animationColor){
            case PRIMARY_COLOR:
                singleFillColors[0] = hsbViaJoystick(singleFill.getColors()[0]);
                break;
            case SECONDARY_COLOR:
                singleFillColors[1] = hsbViaJoystick(singleFill.getColors()[1]);
                break;
            case TERTIARY_COLOR:
                singleFillColors[2] = hsbViaJoystick(singleFill.getColors()[2]);
                break;
        }

        toggleThroughColors(gamepad1.yWasPressed(),true);

        singleFill.setColors(singleFillColors);

        singleFill.setStartIndex(startPoint);
        singleFill.setStopIndex(endPoint);
        singleFill.setBrightness(brightness);
        singleFill.setSpeed(speed);
        singleFill.setPeriod(period);

        if(isBeingInserted){
            prism.insertAndUpdateAnimation(selectedLayer.layerHeight, singleFill);
        } else {
            prism.updateAnimationFromIndex(selectedLayer.layerHeight);
        }

        if(showTelemetry){
            telemetry.addLine("Selected Animation: Single Fill");
            telemetry.addLine("");
            showHsbTelemetry();
            telemetry.addLine(hsbTelemetry);
            telemetry.addLine("");
            telemetry.addLine("Click the Y button to switch between setting the primary, secondary, and tertiary color");
            telemetry.addLine(animationColor.toString());
            telemetry.addLine("");
            telemetry.addLine("Press A to Continue");
            telemetry.addLine("Press B to go back");
        }
    }

    public void configureRainbowSnakes(boolean showTelemetry, boolean isBeingInserted){
        float[] hues = hueViaJoystick(rainbowSnakes.getStartHue(),rainbowSnakes.getStopHue());
        rainbowSnakes.setStartHue(hues[0]);
        rainbowSnakes.setStopHue(hues[1]);

        rainbowSnakes.setStartIndex(startPoint);
        rainbowSnakes.setStopIndex(endPoint);
        rainbowSnakes.setBrightness(brightness);
        rainbowSnakes.setSpeed(speed);

        if(isBeingInserted){
            prism.insertAndUpdateAnimation(selectedLayer.layerHeight, rainbowSnakes);
        } else {
            prism.updateAnimationFromIndex(selectedLayer.layerHeight);
        }

        if(showTelemetry){
            telemetry.addLine("Selected Animation: Rainbow Snakes");
            telemetry.addLine("Move the joysticks to configure the range of the rainbow");
            telemetry.addLine("Left Joystick X (side-to-side) changes the start hue");
            telemetry.addLine("Right Joystick x (side-to-side) changes the end hue");
            telemetry.addLine("");
            telemetry.addLine(hueTelemetry);
            telemetry.addLine("");
            telemetry.addLine("Press A to Continue");
            telemetry.addLine("Press B to go back");
        }
    }

    public void configurePoliceLights(boolean showTelemetry, boolean isBeingInserted){
        switch (animationColor){
            case PRIMARY_COLOR:
                policeLights.setPrimaryColor(hsbViaJoystick(policeLights.getPrimaryColor()));
                break;
            case SECONDARY_COLOR:
                policeLights.setSecondaryColor(hsbViaJoystick(policeLights.getSecondaryColor()));
                break;
            case TERTIARY_COLOR:
                policeLights.setTertiaryColor(hsbViaJoystick(policeLights.getTertiaryColor()));
                break;
        }

        toggleThroughColors(gamepad1.yWasPressed(),true);

        switch (policeLightsStyle){
            case Style1:
                if(gamepad1.rightStickButtonWasPressed()){
                    policeLightsStyle = PoliceLights.PoliceLightsStyle.Style2;
                }
                break;
            case Style2:
                if(gamepad1.rightStickButtonWasPressed()){
                    policeLightsStyle = PoliceLights.PoliceLightsStyle.Style3;
                }
                break;
            case Style3:
                if(gamepad1.rightStickButtonWasPressed()){
                    policeLightsStyle = PoliceLights.PoliceLightsStyle.Style1;
                }
                break;
        }

        policeLights.setStartIndex(startPoint);
        policeLights.setStopIndex(endPoint);
        policeLights.setBrightness(brightness);
        policeLights.setPoliceLightsStyle(policeLightsStyle);
        policeLights.setPeriod(period);

        if(isBeingInserted){
            prism.insertAndUpdateAnimation(selectedLayer.layerHeight, policeLights);
        } else {
            prism.updateAnimationFromIndex(selectedLayer.layerHeight);
        }

        if(showTelemetry){
            telemetry.addLine("Selected Animation: Police Lights");
            telemetry.addLine("");
            telemetry.addLine("Toggle between styles by clicking the right joystick");
            telemetry.addData("Police Lights Style: ",policeLightsStyle);
            telemetry.addLine("");
            showHsbTelemetry();
            telemetry.addLine(hsbTelemetry);
            telemetry.addLine("");
            telemetry.addLine("Click the Y button to switch between setting the primary, secondary, and tertiary color");
            telemetry.addLine(animationColor.toString());
            telemetry.addLine("");
            telemetry.addLine("Press A to Continue");
            telemetry.addLine("Press B to go back");
        }
    }

    public void resetStoredAnimations(){
        Layers.LAYER_0.animationType = AnimationType.NONE;
        Layers.LAYER_1.animationType = AnimationType.NONE;
        Layers.LAYER_2.animationType = AnimationType.NONE;
        Layers.LAYER_3.animationType = AnimationType.NONE;
        Layers.LAYER_4.animationType = AnimationType.NONE;
        Layers.LAYER_5.animationType = AnimationType.NONE;
        Layers.LAYER_6.animationType = AnimationType.NONE;
        Layers.LAYER_7.animationType = AnimationType.NONE;
        Layers.LAYER_8.animationType = AnimationType.NONE;
        Layers.LAYER_9.animationType = AnimationType.NONE;
    }

    /*
     * Reasonably, you might be wondering why I'm configuring the colors in HSB/HSL instead
     * of RGB - The sane choice. And it's all to create a more intuitive user experience.
     * Having the user create a color by combining sliders of red, green, and blue is very true
     * to the colorspace we are actually working in, but folks I tried this on found it very
     * difficult. Hue/Saturation/Brightness allows us to create one slider for each three intuitive
     * variables. Hue changes the color, saturation changes the intensity of the color, and brightness
     * changes, well the brightness.
     * Actually implementing this isn't very clean, but the result for the user is a better experience.
     */
    public Color hsbViaJoystick(Color previousColor){
        final float HUE_JOYSTICK_SCALAR = 5;
        final float SATURATION_JOYSTICK_SCALAR = 0.05F;
        final float BRIGHTNESS_JOYSTICK_SCALAR = 0.05f;

        float[] hsb = new float[3]; // Android graphics library wants an array containing RGB values.
        android.graphics.Color.RGBToHSV(previousColor.red,previousColor.green,previousColor.blue,hsb);

        /*
         * Here we let the user increase or decrease H, S, or B with the joystick.
         * Pushing the stick more moves the value more.
         */
        hsb[0] = Math.max(Math.min(hsb[0] +(gamepad1.left_stick_x*HUE_JOYSTICK_SCALAR), 360),0);
        hsb[1] = Math.max(Math.min(hsb[1] +(gamepad1.right_stick_x*SATURATION_JOYSTICK_SCALAR), 1),0);
        hsb[2] =  Math.max(Math.min(hsb[2] +(-gamepad1.right_stick_y*BRIGHTNESS_JOYSTICK_SCALAR), 0.98f),0);

        /*
         * Here we create an integer where the Android graphics library will store each component of
         * our RGB color one-after-another. I hope we can agree that this is cursed.
         */
        int colorInt = android.graphics.Color.HSVToColor(hsb);
        Color color = new Color(0,0,0); // Create a new color to return.

        /*
         * One of the big downsides in this multi-color model is that some behavior isn't very
         * intuitive. In this case as you approach a saturation of 1, the number of hues you can
         * display are limited to very pure versions of red, green, and blue. To avoid this we
         * limit the maximum saturation to 0.98 (this limit happens above) and to reduce confusion,
         * we display this to the user over telemetry as 1.0.
         */
        if(hsb[2] > 0.97){
            hsb[2] = 1.0f;
        }
        hsbTelemetry = String.format("Hue/Saturation/Brightness: %4.2f %4.2f %4.2f", hsb[0], hsb[1], hsb[2]);

        color.red = android.graphics.Color.red(colorInt);
        color.green = android.graphics.Color.green(colorInt);
        color.blue = android.graphics.Color.blue(colorInt);
        return color;
    }

    public float[] hueViaJoystick(float previousStartHue, float previousEndHue){
        final float HUE_JOYSTICK_SCALAR = 5;
        float[] hues = new float[2];

        hues[0] = Math.max(Math.min(previousStartHue +(gamepad1.left_stick_x*HUE_JOYSTICK_SCALAR), 360),0);
        hues[1] = Math.max(Math.min(previousEndHue +(gamepad1.right_stick_x*HUE_JOYSTICK_SCALAR),360),0);

        hueTelemetry = String.format("Start hue/end hue: %4.2f %4.2f", hues[0], hues[1]);

        return hues;
    }

    public void showHsbTelemetry(){
        telemetry.addLine("Move the joysticks to configure the color of the LEDs");
        telemetry.addLine("Left Joystick X (side-to-side) Changes the Hue of a color");
        telemetry.addLine("Right Joystick X (side-to-side) changes the Saturation");
        telemetry.addLine("Right Joystick Y (up-and-down) changes the Brightness");
        telemetry.addLine("");
    }

    public void toggleThroughColors(boolean button, boolean thirdColor){
        if(button){
            switch (animationColor){
                case PRIMARY_COLOR:
                    animationColor = AnimationColor.SECONDARY_COLOR;
                    break;
                case SECONDARY_COLOR:
                    if(thirdColor){
                        animationColor = AnimationColor.TERTIARY_COLOR;
                    } else{
                        animationColor = AnimationColor.PRIMARY_COLOR;
                    }
                    break;
                case TERTIARY_COLOR:
                    animationColor = AnimationColor.PRIMARY_COLOR;
                    break;
            }
        }
    }
}
