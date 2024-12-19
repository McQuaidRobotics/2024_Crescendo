package com.igknighters.commands.autos;

public enum Waypoints {
    ;

    public String to(Waypoints wp) {
        return this.name() + "to" + wp.name();
    }
}
