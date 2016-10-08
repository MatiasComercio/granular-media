package ar.edu.itba.ss.granularmedia.services;

import java.util.Random;

public class RandomService {
  private static final Random random = new Random();

  /**
   * Don't let anyone instantiate this class.
   */
  private RandomService() {}

  /**
   * Gets a new pseudo-aleatory random double between the min (inclusive) and max (exclusive) values
   * @param min the min value
   * @param max the max value
   * @return a value between the min (inclusive) and the max (exclusive) value
   */
  public static double randomDouble(final double min, final double max) {
    return min + random.nextDouble() * (max-min);
  }
}
