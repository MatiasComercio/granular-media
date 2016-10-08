package ar.edu.itba.ss.granularmedia.services;

import ar.edu.itba.ss.granularmedia.models.Particle;

import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

// Class and methods package-private as they will be used only within the 'services' package
abstract class Space2DMaths {
  static double distanceBetween(final Particle p1, final Particle p2) {
    return sqrt(pow(p2.x() - p1.x(), 2) + pow(p2.y() - p1.y(), 2)) - p1.radio() - p2.radio();
  }
}
