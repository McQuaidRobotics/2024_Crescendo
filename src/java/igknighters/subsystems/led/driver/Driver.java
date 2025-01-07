package igknighters.subsystems.led.driver;

import igknighters.subsystems.Component;
import igknighters.subsystems.led.LedAnimations.PartialAnimation;
import monologue.Annotations.Log;

public abstract class Driver extends Component {
  @Log protected PartialAnimation[] animations = new PartialAnimation[0];

  /**
   * Sets the animations to be played by the LED strip
   *
   * @param animations The animations to be played
   */
  public abstract void animate(PartialAnimation[] animations);
}
