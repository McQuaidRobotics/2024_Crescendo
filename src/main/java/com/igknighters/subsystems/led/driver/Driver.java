package com.igknighters.subsystems.led.driver;

import com.igknighters.subsystems.Component;
import com.igknighters.subsystems.led.LedAnimations.PartialAnimation;

import monologue.Annotations.Log;

public abstract class Driver extends Component {
    @Log protected PartialAnimation[] animations = new PartialAnimation[0];

    @Override
    public String getOverrideName() {
        return "Driver";
    }

    /**
     * Sets the animations to be played by the LED strip
     * 
     * @param animations The animations to be played
     */
    public abstract void animate(PartialAnimation[] animations);
}
