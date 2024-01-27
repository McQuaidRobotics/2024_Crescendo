package com.igknighters;

import java.util.HashMap;

import javax.naming.ldap.HasControls;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class LED {

    private static LED instance;

    public static LED getInstance() {
        if (instance == null) {
            instance = new LED();
        }
        return instance;
    }

    private final static int NUMLEDS = 8;
    private static int numPatterns = 1;
    private static Animation animDisabled = new ColorFlowAnimation(0, 0, 255, 0, 0.1, NUMLEDS/numPatterns, Direction.Forward);
    private static Animation animTeleOp = new ColorFlowAnimation(0, 255, 0, 0, 0.1, NUMLEDS/numPatterns, Direction.Forward);
    private static Animation animAuto = new RainbowAnimation(1.0, 0.2, NUMLEDS/numPatterns);
    private static Animation animTest = new ColorFlowAnimation(255, 0, 255, 0, 0.1, NUMLEDS/numPatterns, Direction.Forward);
    private static Animation anim20sLeft = new SingleFadeAnimation(255, 255, 255);
    private static Animation animShooting = new ColorFlowAnimation(255, 107, 0, 0, 0.1, NUMLEDS/numPatterns, Direction.Forward, 4); // McQ gold for now
    private static Animation animBooting = new StrobeAnimation(255, 0, 255, 0, 0.1, NUMLEDS/numPatterns);
    private static Animation animNone = null;
    // private static Animation[] animations = 
    // new Animation[]{animDisabled,
    // animTeleOp, 
    // animAuto, 
    // animTest, 
    // anim20sLeft, 
    // animShooting, 
    // animBooting,
    // animNone}; 
    private static HashMap<String, Animation> hashAnims = new HashMap<String, Animation>();
    
    // public enum LedAnimations {
    //     DISABLED(animations[0]),
    //     TELEOP(animations[1]),
    //     AUTO(animations[2]),
    //     TEST(animations[3]),
    //     _20S_LEFT(animations[4]),
    //     SHOOTING(animations[5]),
    //     BOOTING(animations[6]),
    //     DEFAULT(animations[7]);

    //     private final Animation anim;

    //     public Animation getAnim(){
    //         return anim;
    //     }
        
    //     private LedAnimations(Animation anim) {
    //         this.anim=anim;
    //     }
    // }
    
    
    
    private Timer timer;
    private Double duration;
    // private LedAnimations animation;
    private Animation testAnim;
    private Animation animation2;
    private Animation lastAnimation;
    private CANdle candle;
    private CANdleConfiguration config;
    private XboxController controller = new XboxController(0);

    public LED() {
        hashAnims.put("disabled", animDisabled);
        hashAnims.put("teleop", animTeleOp);
        hashAnims.put("auto", animAuto);
        hashAnims.put("test", animTest);
        hashAnims.put("20s", anim20sLeft);
        hashAnims.put("shooting", animShooting);
        hashAnims.put("booting", animBooting);
        hashAnims.put("none", animNone);
        this.timer = new Timer();
        // this.animation = LedAnimations.DISABLED;
        // this.animation2 = LedAnimations.DEFAULT;
        // this.lastPattern = LedAnimations.DISABLED;
        this.duration = 0.0;
        candle = new CANdle(51);
        config = new CANdleConfiguration();
        config.v5Enabled = true;
        config.stripType = LEDStripType.RGB;
        config.brightnessScalar = 0.1;
        config.disableWhenLOS = true;
        candle.configAllSettings(config);
        sendAnimation(testAnim);

    }


    //     new Trigger(DriverStation::isFMSAttached)
    //             .and(() -> Math.abs(DriverStation.getMatchTime() - 30.0) < 0.2)
    //             .onTrue(new InstantCommand(() -> this.setLed(LedAnimations._20S_LEFT, 2)));

    // }

    
    // private void sendAnimation(LedAnimations anim) {
    //     candle.animate(anim.getAnim());
    // }
    
    private void sendAnimation(Animation anim){
        candle.animate(anim);
    }
    
    // private void sendMultiAnimation(LedAnimations anim1, LedAnimations anim2){
    //     candle.animate(anim1.getAnim(), 0);
    //     candle.animate(anim2.getAnim(), 1);
    // }
    private void sendMultiAnimation(Animation anim1, Animation anim2){
        candle.animate(anim1, 0);
        candle.animate(anim2, 1);
    }

    // public void setLed(LedAnimations anim, double seconds) {
    //     this.timer.restart();
    //     this.animation = anim;
    //     this.duration = seconds;
    //}


    // public void setLed(LedAnimations anim) {
    //     setLed(anim, Double.POSITIVE_INFINITY);
    // }

    public void setDefault() {
        numPatterns = 1;
        animDisabled = new ColorFlowAnimation(0, 0, 255, 0, 0.1, NUMLEDS/numPatterns, Direction.Forward);
        animTeleOp = new ColorFlowAnimation(0, 255, 0, 0, 0.1, NUMLEDS/numPatterns, Direction.Forward);
        animAuto = new RainbowAnimation(1.0, 0.2, NUMLEDS/numPatterns);
        animTest = new ColorFlowAnimation(255, 0, 255, 0, 0.1, NUMLEDS/numPatterns, Direction.Forward);
        anim20sLeft = new SingleFadeAnimation(255, 255, 255);
        animShooting = new ColorFlowAnimation(255, 107, 0, 0, 0.1, NUMLEDS/numPatterns, Direction.Forward, 4); // McQ gold for now
        animBooting = new StrobeAnimation(255, 0, 255, 0, 0.1, NUMLEDS/numPatterns);
        this.duration = 0.0;
        // animations = 
        // new Animation[]{animDisabled,
        // animTeleOp, 
        // animAuto, 
        // animTest, 
        // anim20sLeft, 
        // animShooting, 
        // animBooting,
        // animNone};
        hashAnims.put("disabled", animDisabled);
        hashAnims.put("teleop", animTeleOp);
        hashAnims.put("auto", animAuto);
        hashAnims.put("test", animTest);
        hashAnims.put("20s", anim20sLeft);
        hashAnims.put("shooting", animShooting);
        hashAnims.put("booting", animBooting);
        hashAnims.put("none", animNone); 
        if (DriverStation.isDisabled()) {
            //this.animation = LedAnimations.DISABLED;
            this.testAnim = hashAnims.get("disabled");
        } else if (DriverStation.isAutonomous()) {
            //this.animation = LedAnimations.AUTO;
            this.testAnim = hashAnims.get("auto");
        } else {
            //this.animation = LedAnimations.TELEOP;
            this.testAnim = hashAnims.get("teleop");
        }
        
    }

    public void shooting(Animation anim) {
        if (timer.hasElapsed(duration)){
            this.animation2=hashAnims.get("none");
        }
        else{
            this.animation2 = hashAnims.get("shooting");
            sendMultiAnimation(anim, animation2);
        }
    }

    public void run() {
        if (timer.hasElapsed(duration)) {
            setDefault();
            animation2 = hashAnims.get("none");
        }
        if (testAnim != lastAnimation || duration == 0) {
            //sendMultiAnimation(animation, animation2);
            sendMultiAnimation(testAnim, animation2);
            lastAnimation = testAnim;
        }
        if (controller.getAButton()){
            numPatterns=2;
            this.timer.restart();
            duration = 2.5;
            hashAnims.put("teleop", new ColorFlowAnimation(255, 0, 255, 0, 0.1, NUMLEDS/numPatterns, Direction.Forward));
            testAnim = hashAnims.get("teleop");
            shooting(testAnim);
        }

    }
    

}
