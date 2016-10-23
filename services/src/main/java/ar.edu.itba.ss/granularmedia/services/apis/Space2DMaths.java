package ar.edu.itba.ss.granularmedia.services.apis;

import ar.edu.itba.ss.granularmedia.models.Particle;
import ar.edu.itba.ss.granularmedia.models.Vector2D;
import ar.edu.itba.ss.granularmedia.models.Wall;

import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

// Class and methods package-private as they will be used only within the 'services' package
public abstract class Space2DMaths {
  private static final int NORMAL = 0;
  private static final int TANGENTIAL = 1;

  public static Vector2D nullVector() {
    return Vector2D.builder(0, 0).build();
  }

  public static double distanceBetween(final Particle p1, final Particle p2) {
    return sqrt(pow(p2.x() - p1.x(), 2) + pow(p2.y() - p1.y(), 2)) - p1.radio() - p2.radio();
  }

  public static double superpositionBetween(final Particle particle, final Particle neighbour) {
    // superposition of two particles is the opposite of the distance between them
    return - distanceBetween(particle, neighbour);
  }

  /**
   *
   * @param p a particle
   * @param w a wall
   * @return superposition value >= 0 if there is a collision indeed; otherwise, a negative value is returned
   */
  // Modified from:
  // https://code.tutsplus.com/tutorials/quick-tip-collision-detection-between-a-circle-and-a-line-segment--active-10632
  public static double superpositionBetween(final Particle p, final Wall w) {
    //calculating wall's perpendicular distance to particle
    final Vector2D c1Particle = p.r0().sub(w.c1());

    // get both normal and tangent versors
    final Vector2D[] normalAndTangentialVersors = normalAndTangentialVersors(p, w);
    final Vector2D normalVersor = normalAndTangentialVersors[NORMAL];
    final Vector2D tangentialVersor = normalAndTangentialVersors[TANGENTIAL];

    final double normalProjectionModule = Math.abs(dotProduct(c1Particle, normalVersor));
    final double tangentialProjectionModule = Math.abs(dotProduct(c1Particle, tangentialVersor));

    final double distance = normalProjectionModule - p.radio();
    final double superposition = - distance;
    if (    distance <= 0
            && dotProduct(c1Particle, w.asVector().toVersor()) > 0
            && tangentialProjectionModule < w.asVector().norm2()) {
      // collision detected => superposition value is representative of the collision
      return superposition;
    }

    // particle is in the same line as wall, but out of its bounds ==> superposition is not representative => return < 0
    return -1;
  }

  /**
   * Calculates the normal and tangential versors of the {@code wall} relative to the particle's position vector
   * @param p a particle
   * @param w a wall
   * @return normal and tangential versors, in that order;
   */
  public static Vector2D[] normalAndTangentialVersors(final Particle p, final Wall w) {
    final Vector2D tangentialVersor = w.tangentialVersor();
    final Vector2D leftNormalVersor = w.leftNormalVersor();
    final Vector2D rightNormalVersor = w.rightNormalVersor();
    final Vector2D r0 = p.r0();
    final Vector2D c1 = w.c1();
    final Vector2D c1r0 = r0.sub(c1);

    /*
        The corresponding normal versor for this particle is the one pointing from the particle to the wall.
        This occurs when the dot product between the particle's position and the correct normal vector is negative.
        If leftNormalVersor should happen to be the correct one to pick, then, the tangential versor should be rotated
        so as to match the right hand rule.
        If rightNormalVersor should happen to be the correct, the tangential versor just obtain already satisfies
        the right hand rule.
     */
    if (dotProduct(c1r0, leftNormalVersor) < 0) { // should choose left normal versor
      return new Vector2D[] { leftNormalVersor, tangentialVersor.times(-1) };
    }
    // should choose right normal versor
    return new Vector2D[] { rightNormalVersor, tangentialVersor };
  }

  /**
   * Calculates the normal and tangential versors of the vector {@code j} relative to the vector {@code i}.
   * <P>
   * Normal versor will be pointing from the {@code i} particle to the {@code j} particle.
   * <P>
   * Tangent versor will be pointing towards the direction calculated with the right hand rule.
   * <P>
   * For example, if i is (0, 0) and j is (1, 0), normalVersor = (1, 0) and tangentVersor = (0, 1).
   * @param i a 2D vector
   * @param j another 2D vector
   * @return normal and tangential versors, in that order; null if {@code i} = {@code j};
   * @implNote normalVector = (<b>r</b>j - <b>r</b>i) / |<b>r</b>j - <b>r</b>i|
   */
  public static Vector2D[] normalAndTangentialVersors(final Vector2D i, final Vector2D j) {
    final Vector2D relativeVector = relativeVector(i, j);
    final Vector2D normalVersor = relativeVector.toVersor();
    if (normalVersor == null) {
      return null;
    }

    final Vector2D tangentialVersor = Vector2D.builder(-normalVersor.y(), normalVersor.x()).build();

    return new Vector2D[] { normalVersor, tangentialVersor };
  }

  /**
   * Calculates the relative vector from {@code i} to {@code j}.
   * <P>
   * For example, if i is (0, 0) and j is (1, 0), relativeVector = (1, 0)
   * @param i a 2D vector
   * @param j another 2D vector
   * @return relative vector from {@code i} to {@code j}
   * @implNote relativeVector = (<b>r</b>j - <b>r</b>i)
   */
  public static Vector2D relativeVector(final Vector2D i, final Vector2D j) {
    return j.sub(i);
  }

  @SuppressWarnings("unused")
  public static double distanceBetween(final Vector2D i, final Vector2D j) {
    return relativeVector(i, j).norm2();
  }

  public static double dotProduct(final Vector2D i, final Vector2D j) {
    return i.x() * j.x() + i.y() * j.y();
  }
}
