package ar.edu.itba.ss.granularmedia.services.gear;

import ar.edu.itba.ss.granularmedia.models.Particle;

import java.util.Collection;

/**
 * Data manager of a Gear Predictor Corrector numeric integration method of order 5
 */
public abstract class Gear5SystemData extends GearSystemData {
  /**
   * Order of the data manager
   */
  private static final int ORDER = 5;
  /**
   * Size of the memory structures to be used when using this data manager
   */
  private static final int S_VECTORS = ORDER + 1;

  private static final long[] factorial = new long[S_VECTORS];
  private static final double[] alpha = new double[S_VECTORS];

  static {
    for(int i = 0; i < S_VECTORS; i++) {
      factorial[i] = staticFactorial(i);
    }

    //Alpha values taken from class presentation
    alpha[0] = 3.0/16.0;
    alpha[1] = 251.0/360.0;
    alpha[2] = 1.0;
    alpha[3] = 11.0/18.0;
    alpha[4] = 1.0/6.0;
    alpha[5] = 1.0/60.0;
  }

  public Gear5SystemData(final Collection<Particle> particles) {
    super(particles);
  }

  @Override
  protected int order() {
    return ORDER;
  }

  @Override
  protected int sVectors() {
    return S_VECTORS;
  }

  @Override
  /* package-private */ long factorial(final int n) {
    return factorial[n];
  }

  @Override
  /* package-private */ double alpha(final int n) {
    return alpha[n];
  }
}
