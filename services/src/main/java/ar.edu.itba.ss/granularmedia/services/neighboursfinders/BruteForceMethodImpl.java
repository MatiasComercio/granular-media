package ar.edu.itba.ss.granularmedia.services.neighboursfinders;

import ar.edu.itba.ss.granularmedia.interfaces.NeighboursFinder;
import ar.edu.itba.ss.granularmedia.models.Particle;
import ar.edu.itba.ss.granularmedia.services.apis.Space2DMaths;

import java.util.*;

public class BruteForceMethodImpl implements NeighboursFinder {

  @Override
  public Map<Particle, Collection<Particle>> run(Collection<Particle> particles,
                                                 double L, double W, int M1, int M2, double rc, boolean periodicLimit) {

    final List<Particle> pointsAsList = new ArrayList<>(particles);
    final Map<Particle, Collection<Particle>> collisionPerParticle = new HashMap<>(particles.size());

    particles.forEach(point -> {
      // add the point to the map to be returned, with a new empty set
      collisionPerParticle.put(point, new HashSet<>());
    });

    if (!periodicLimit) {
      calculateCollisions(collisionPerParticle, pointsAsList, rc);
    }

    return collisionPerParticle;
  }

  private void calculateCollisions(final Map<Particle, Collection<Particle>> collisionPerParticle,
                                   final List<Particle> pointsAsList, final double rc) {
    double distance;

    for (int i = 0; i < pointsAsList.size(); i++) {
      for (int j = 0; j < pointsAsList.size(); j++) {
        distance = Space2DMaths.distanceBetween(pointsAsList.get(i),
                pointsAsList.get(j));
        if (distance <= rc && !pointsAsList.get(i).equals(pointsAsList.get(j))) {
          collisionPerParticle.get(pointsAsList.get(i)).add(pointsAsList.get(j));
          collisionPerParticle.get(pointsAsList.get(j)).add(pointsAsList.get(i));
        }
      }
    }
  }
}
