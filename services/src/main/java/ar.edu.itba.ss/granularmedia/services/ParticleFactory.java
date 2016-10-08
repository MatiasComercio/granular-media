package ar.edu.itba.ss.granularmedia.services;

import ar.edu.itba.ss.granularmedia.models.Particle;

import java.util.HashSet;
import java.util.Set;

public class ParticleFactory {
  private static ParticleFactory particleFactory;

  private ParticleFactory() {
  }

  public static ParticleFactory getInstance() {
    if (particleFactory == null) {
      particleFactory = new ParticleFactory();
    }
    return particleFactory;
  }

  /**
   * Creates a new particle with the given x and y position, and all default attributes
   * @param x x position
   * @param y y position
   * @return the created particle
   */
  public Particle create(final double x, final double y) {
    return Particle.builder(x, y).build();
  }

  /**
   * Generates random positioned particles based on the given parameters.
   * Collisions are accepted or not depending the given parameter.
   *
   * @param leftBottomParticle the particle at that corner of the area to where the particles must belong ; null if random
   * @param rightTopParticle the particle at that corner of the area to where the particles must belong ; null if random
   * @param radios the particle's radios
   *
   * @param overlapAllowed whether the particles can collide or not
   * @param maxTries how many times the function will try to generate non-colliding particles - consecutively.
   *                 If this limit is reached, the set as is at that moment is returned
   * @return a set containing the generated particles - could have less than amount particles
   * due to having reach maxTries without being able to find an empty place where the particle does not collide
   */
  public Set<Particle> randomPoints(final Particle leftBottomParticle,
                             final Particle rightTopParticle,
                             final double[] radios,
                             final boolean overlapAllowed,
                             final int maxTries) {
    final double minX, minY, maxX, maxY;
    if (leftBottomParticle != null) {
      minX = leftBottomParticle.x();
      minY = leftBottomParticle.y();
    } else {
      minX = minY = Double.MIN_VALUE;
    }

    if (rightTopParticle != null) {
      maxX = rightTopParticle.x();
      maxY = rightTopParticle.y();
    } else {
      maxX = maxY = Double.MAX_VALUE;
    }

    final int amount = radios.length;

    final Set<Particle> generatedParticles = new HashSet<>(amount);


    for (final double radio : radios) {
      final Particle currentParticle;

      // adapt max and min (x,y) so that current particle does not overlaps the silo's border
      final double currMinX, currMaxX, currMinY, currMaxY;
      currMinX = minX + radio;
      currMaxX = maxX - radio;
      currMinY = minY + radio;
      currMaxY = maxY - radio;

      if (overlapAllowed) {
        currentParticle = createOverlappedParticle(currMinX, currMaxX, currMinY, currMaxY, radio);
      } else {
        currentParticle = createNonOverlappedParticle(
                currMinX, currMaxX, currMinY, currMaxY, radio, generatedParticles, maxTries);
        if (currentParticle == null) { // could not generate a new particle that does not overlap
          return generatedParticles;
        }
      }

      // for sure that the particle is not at the set; if it were, it would have overlapped with itself
      generatedParticles.add(currentParticle);
    }

    return generatedParticles;
  }

  /**
   * Creates a new particle that can overlap all the previous generated ones
   * @param minX min x position the particle can have
   * @param maxX max x position the particle can have
   * @param minY min y position the particle can have
   * @param maxY max y position the particle can have
   * @param radio particle's radio
   * @return the new particle
   */
  private Particle createOverlappedParticle(final double minX, final double maxX,
                                            final double minY, final double maxY,
                                            final double radio) {
    return createParticle(minX, maxX, minY, maxY, radio);
  }

  /**
   *
   * Creates a new particle checking that it does not overlap with any of the previous generated ones.
   * <P>
   * It tries to create such a particle a number of {@code maxTries} times.
   * <P>
   * If fail to do this, null is returned.
   * @param minX min x position the particle can have
   * @param maxX max x position the particle can have
   * @param minY min y position the particle can have
   * @param maxY max y position the particle can have
   * @param radio particle's radio
   * @param generatedParticles all the previous generated particles
   * @param maxTries how many times it will try to create a new non-overlapping particle
   * @return the created particle if it does not overlap with any of the previous generated ones and
   * {@code maxTries}; null otherwise
   */
  private Particle createNonOverlappedParticle(final double minX, final double maxX,
                                               final double minY, final double maxY,
                                               final double radio,
                                               final Set<Particle> generatedParticles,
                                               final int maxTries) {
    int tries = 0;
    Particle createdParticle;
    do {
      createdParticle = createParticle(minX, maxX, minY, maxY, radio);

      tries++;
      if (tries > maxTries) {
        return null;
      }
    } while (overlaps(createdParticle, generatedParticles));

    return createdParticle;
  }

  /**
   * Creates a particle within the given range of x and y values, choose randomly, and the given radio
   * @param minX min x
   * @param maxX max x
   * @param minY min y
   * @param maxY max y
   * @param radio radio
   * @return the new particle created with the specified criteria & parameters
   */
  private Particle createParticle(final double minX, final double maxX,
                                final double minY, final double maxY,
                                final double radio) {
    double pX = RandomService.randomDouble(minX, maxX);
    double pY = RandomService.randomDouble(minY, maxY);
    double pR = radio <= -1 ? 0 : radio;

    return Particle.builder(pX, pY).radio(pR).build();
  }

  /**
   * Checks whether the just created particles overlaps all the previous created particles
   * @param currentParticle the just created particle
   * @param generatedParticles all previous particles
   * @return true if current particle overlaps any of the previous created ones; false otherwise
   */
  private boolean overlaps(final Particle currentParticle,
                           final Set<Particle> generatedParticles) {
    for (Particle each : generatedParticles) {
      if (Space2DMaths.distanceBetween(each, currentParticle) < 0) {
        return true;
      }
    }

    return false;
  }
}
