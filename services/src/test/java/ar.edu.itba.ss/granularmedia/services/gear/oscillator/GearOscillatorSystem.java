package ar.edu.itba.ss.granularmedia.services.gear.oscillator;

import ar.edu.itba.ss.granularmedia.interfaces.NumericIntegrationMethod;
import ar.edu.itba.ss.granularmedia.interfaces.SystemData;
import ar.edu.itba.ss.granularmedia.interfaces.TimeDrivenSimulationSystem;
import ar.edu.itba.ss.granularmedia.models.Particle;
import ar.edu.itba.ss.granularmedia.models.Vector2D;
import ar.edu.itba.ss.granularmedia.services.gear.Gear5SystemData;
import ar.edu.itba.ss.granularmedia.services.gear.GearPredictorCorrector;

import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;

public class GearOscillatorSystem implements TimeDrivenSimulationSystem {

  private final NumericIntegrationMethod<Gear5SystemData> numericIntegrationMethod;
  private final Gear5SystemData systemData;

  public GearOscillatorSystem(final double mass, final double r, final double k, final double gamma) {

    /*
      template steps:
      - instantiate all needed particles
      - create system data container with system's particles and constants
      - initialize the numeric integration method and the corresponding System's data structure
     */

    // Calculate particle's initial values
    final double beta = gamma / (2 * mass);
    final double initialVx = -beta;
    final Particle particle = Particle.builder(r, 0)
            .mass(mass)
            .vx(initialVx)
            .build();

    final Collection<Particle> particles = new HashSet<>();
    particles.add(particle);

    // Numeric Integration Method initialization
    this.systemData = new OscillatorGear5SystemData(particles, k, gamma);

    this.numericIntegrationMethod = new GearPredictorCorrector<>();
  }

  @Override
  public SystemData getSystemData() {
    return this.systemData;
  }

  @Override
  public void evolveSystem(final double dt) {
    numericIntegrationMethod.evolveSystem(this.systemData, dt);
  }

  private static class OscillatorGear5SystemData extends Gear5SystemData {
    private final double k;
    private final double gamma;

    private OscillatorGear5SystemData(final Collection<Particle> particles,
                                     final double k, final double gamma) {
      super(particles);

      // Save constant parameters
      this.k = k;
      this.gamma = gamma;

      particles.forEach(this::initParticle);
    }

    @Override
    protected Map<Integer, Vector2D> setInitialDerivativeValues(final Particle particle) {
      final Map<Integer, Vector2D> initialDerivativeValues = new HashMap<>(sVectors());
      initialDerivativeValues.put(0, Vector2D.builder(particle.x(), particle.y()).build());
      initialDerivativeValues.put(1, Vector2D.builder(particle.vx(), particle.vy()).build());

      for(int i = 2; i <= order(); i++) {
        final Vector2D rPrev2 = initialDerivativeValues.get(i-2);
        final Vector2D rPrev1 = initialDerivativeValues.get(i-1);
        final Vector2D term1 = rPrev2.times(-k);
        final Vector2D term2 = rPrev1.times(gamma);
        final Vector2D rCurr = (term1.sub(term2)).div(particle.mass());
        initialDerivativeValues.put(i, rCurr);
      }

      return initialDerivativeValues;
    }

    @Override
    protected Vector2D getForceWithPredicted(final Particle particle) {
      final Vector2D rPredicted0 = getPredictedR(particle, 0);
      final Vector2D term1 = rPredicted0.times(-k);
      final Vector2D rPredicted1 = getPredictedR(particle, 1);
      final Vector2D term2 = rPredicted1.times(gamma);
      return term1.sub(term2);
    }
  }
}
