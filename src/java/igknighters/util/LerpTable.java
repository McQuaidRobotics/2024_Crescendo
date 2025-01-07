package igknighters.util;

public class LerpTable {
  public static class LerpTableEntry {
    public double x;
    public double y;

    public LerpTableEntry(double x, double y) {
      this.x = x;
      this.y = y;
    }

    public double lerp(LerpTableEntry other, double x) {
      return this.y + (other.y - this.y) * ((x - this.x) / (other.x - this.x));
    }
  }

  private LerpTableEntry[] table;

  public LerpTable(LerpTableEntry... table) {
    this.table = table;
  }

  public double lerp(double x) {
    if (x < table[0].x) {
      return table[0].y;
    } else if (x > table[table.length - 1].x) {
      return table[table.length - 1].y;
    }

    for (int i = 0; i < table.length - 1; i++) {
      if (x >= table[i].x && x <= table[i + 1].x) {
        return table[i].lerp(table[i + 1], x);
      }
    }

    return 0;
  }

  public double lerpKeepSign(double x) {
    return lerp(Math.abs(x)) * Math.signum(x);
  }
}
