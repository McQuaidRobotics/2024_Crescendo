package igknighters.subsystems.led.driver;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import igknighters.subsystems.led.LedAnimations.LedPattern;
import igknighters.subsystems.led.LedAnimations.PartialAnimation;
import monologue.Annotations.Log;

public class CandleDriver extends Driver {

  private final CANdle candle;

  @Log private boolean newAnimations = false;
  private PartialAnimation[] lastAnimations = new PartialAnimation[0];

  public CandleDriver() {
    candle = new CANdle(52);
    var config = new CANdleConfiguration();
    config.v5Enabled = true;
    config.stripType = LEDStripType.RGB;
    config.brightnessScalar = 1.0;
    config.disableWhenLOS = true;
    candle.configAllSettings(config, 1000);
  }

  @Override
  public void animate(PartialAnimation[] animations) {
    this.animations = animations;
    if (animations.length != lastAnimations.length) {
      newAnimations = true;
    } else {
      for (int i = 0; i < animations.length; i++) {
        if (animations[i] != lastAnimations[i]) {
          newAnimations = true;
          break;
        }
      }
    }
  }

  private void execPattern(PartialAnimation partialAnim, int index) {
    int leds = partialAnim.leds();
    int offset = partialAnim.offset();
    if (partialAnim.anim().pattern instanceof LedPattern.Solid solid) {
      candle.setLEDs(solid.r(), solid.g(), solid.b(), solid.w(), offset, leds);
    } else if (partialAnim.anim().pattern instanceof LedPattern.Strobe strobe) {
      candle.animate(
          new StrobeAnimation(
              strobe.r(), strobe.g(), strobe.b(), strobe.w(), strobe.speed(), leds, offset),
          index);
    } else if (partialAnim.anim().pattern instanceof LedPattern.Flow flow) {
      candle.animate(
          new ColorFlowAnimation(
              flow.r(),
              flow.g(),
              flow.b(),
              flow.w(),
              flow.speed(),
              leds,
              flow.backward() ? Direction.Backward : Direction.Forward,
              offset),
          index);
    } else if (partialAnim.anim().pattern instanceof LedPattern.Rainbow rainbow) {
      candle.animate(
          new RainbowAnimation(
              rainbow.brightness(), rainbow.speed(), leds, rainbow.backward(), offset),
          index);
    } else if (partialAnim.anim().pattern instanceof LedPattern.Fire fire) {
      candle.animate(
          new FireAnimation(
              fire.brightness(),
              fire.speed(),
              leds,
              fire.sparking(),
              fire.cooling(),
              fire.backward(),
              offset),
          index);
    } else {
      DriverStation.reportError(
          "Unknown animation type: " + partialAnim.anim().pattern.getClass().getSimpleName(),
          false);
    }
  }

  @Override
  public void periodic() {
    double percent = MathUtil.clamp(RobotController.getBatteryVoltage() - 8.0, 0.0, 4.5) / 4.5;
    int green = (int) (percent * 255);
    int red = (int) ((1 - percent) * 255);
    candle.setLEDs(red, green, 0, 255, 0, 8);
    if (newAnimations) {
      for (int i = 0; i < lastAnimations.length; i++) {
        candle.clearAnimation(i + 1);
      }
      for (int i = 0; i < animations.length; i++) {
        execPattern(animations[i], i + 1);
      }
      newAnimations = false;
      lastAnimations = animations;
    }
  }
}
