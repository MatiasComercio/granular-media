package ar.edu.itba.ss.granularmedia.services;

import ar.edu.itba.ss.granularmedia.models.Particle;
import ar.edu.itba.ss.granularmedia.services.apis.Space2DMaths;
import ar.edu.itba.ss.granularmedia.services.factories.ParticleFactory;
import org.junit.Test;

import java.util.Arrays;
import java.util.Set;

public class ParticleFactoryTest {
  private static final int N_PARTICLES = 500;
  private static final double[] radios;
  private static final double MASS = 0.001;
  private static final double RADIO = 2;
  private static final int MAX_TRIES = 100;
  private static final int MIN_X = 0;
  private static final int MIN_Y = 0;
  private static final int MAX_X = 20;
  private static final int MAX_Y = 20;
  private static final Particle leftBottomParticle = Particle.builder(MIN_X, MIN_Y).build();
  private static final Particle rightTopParticle = Particle.builder(MAX_X, MAX_Y).build();

  static {
    radios = new double[N_PARTICLES];
    Arrays.fill(radios, RADIO);
  }

  /* trivial repetition test */
  @Test
  public void testRandomPoints() {
    // trivial repetition
    for (int i = 0 ; i < 1000 ; i++) {
      wrappedTestRandomPoints();
    }
  }

  private void wrappedTestRandomPoints() {
    final ParticleFactory pF = ParticleFactory.getInstance();

    final Set<Particle> points = pF.randomPoints(leftBottomParticle, rightTopParticle, radios, MASS, false, MAX_TRIES);

    for (Particle p1 : points) {

      // check bounds
      if (p1.x() < MIN_X || p1.y() < MIN_Y || p1.x() >= MAX_X || p1.y() >= MAX_Y) {
        // out of bounds
        assert false; // this will throw an assertion
      }

      // check collision
      for (Particle p2 : points) {
        if (p1.equals(p2)) {
          continue;
        }

        if (Space2DMaths.distanceBetween(p1, p2) < 0) {
          // collision detected
          assert false;
        }
      }
    }
  }
}
