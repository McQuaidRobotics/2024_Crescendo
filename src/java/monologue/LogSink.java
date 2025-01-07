package monologue;

public enum LogSink {
  /** Logs will be sent to NetworkTables and then mirrored to DataLog */
  NT,
  /** Logs will be sent to DataLog only */
  DL,
  /**
   * Will behave like {@link #DL} if {@link Monologue#isBandwidthOptimizationEnabled()} is true,
   * otherwise will behave like {@link #NT}
   */
  OP;

  /**
   * Whether or not to log under the current library/logger flags
   *
   * @param fileOnly If the library is in fileOnly mode
   * @param nt If the logger asking is the nt logger
   * @return
   */
  boolean shouldLog(boolean fileOnly, boolean nt) {
    switch (this) {
      case NT:
        return !nt;
      case DL:
        return !nt;
      case OP:
        if (fileOnly) {
          DL.shouldLog(fileOnly, nt);
        } else {
          NT.shouldLog(fileOnly, nt);
        }
      default:
        return false;
    }
  }
}
