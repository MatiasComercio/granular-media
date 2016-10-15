package ar.edu.itba.ss.granularmedia.interfaces;

import ar.edu.itba.ss.granularmedia.models.Particle;

import java.util.Collection;
import java.util.Map;
import java.util.Set;

public interface NeighboursFinder {

  /**
   * For each point of the given set, this method gets the ones that are colliding with other particles
   * of the collection, considering that a collision is produced when two particles are at distance lower or equal than rc,
   * considering both point's radios.
   * <p>
   * Particles are supposed to be contained on a rectangle of height L and width W ( 0 <= point.x < W && 0 <= point.y < L ).
   * <p>
   * The method will divide that rectangle in M1xM2 cells - each of them with height L/M1 and base W/M2 -, and will use this cells
   * to apply the algorithm.
   * <p>
   *
   * @param particles collection containing the particles for the algorithm
   * @param L length of the height of the rectangle containing all the particles of the set. Must be positive.
   * @param W length of the base of the rectangle containing all the particles of the set. Must be positive.
   * @param M1 number of cells on which the height of the square will be divided. Must be positive.
   * @param M2 number of cells on which the base of the square will be divided. Must be positive.
   * @param rc max distance to consider that two particles are colliding. Must be non negative.
   * @param periodicLimit if the end of a limit cell should be consider as it were from the opposite side
   * @return a map containing as key each of the particles of the set, and a list of the particles with the ones
   * each point collides
   *
   * @throws IllegalArgumentException if M1 <= 0 or M2 <= 0 or rc < 0 or L <= 0 or W <= 0
   * @implNote take into consideration that this algorithm for work requires that
   * the condition L/M1 > rc + r1 + r2 and W/M2 > rc + r1 + r2  and  is met for every pair of particles. However, this condition is not check,
   * so be sure that it is met so as to guaranty that the value returned by this method is valid and real
   */
  Map<Particle, Collection<Particle>> run(Collection<Particle> particles,
                                          double L, double W, int M1, int M2, double rc, boolean periodicLimit);
}
