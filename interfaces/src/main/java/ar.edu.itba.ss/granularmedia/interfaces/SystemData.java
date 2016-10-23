package ar.edu.itba.ss.granularmedia.interfaces;

import ar.edu.itba.ss.granularmedia.models.Particle;

import java.util.Collection;

public interface SystemData {
  /**
   *
   * @return particles contained by this system's data entity
   */
  Collection<Particle> particles();
}
