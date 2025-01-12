package igknighters.commands.autos;

public enum Waypoints {
  R1,
  R2,
  R3,
  R4,
  R5,
  R6,
  R7,
  R8,
  R9,
  R10,
  R11,
  R12,
  S1,
  S2,
  S3,
  S4,
  S5,
  C1,
  C2,
  C3,
  C4,
  C5,
  C6;

  public String to(Waypoints wp) {
    return this.name() + "to" + wp.name();
  }
}
