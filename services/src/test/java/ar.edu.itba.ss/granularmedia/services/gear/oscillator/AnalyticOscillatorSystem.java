package ar.edu.itba.ss.granularmedia.services.gear.oscillator;

import ar.edu.itba.ss.granularmedia.interfaces.SystemData;
import ar.edu.itba.ss.granularmedia.interfaces.TimeDrivenSimulationSystem;
import ar.edu.itba.ss.granularmedia.models.Particle;

import java.util.Collection;
import java.util.HashSet;

import static java.lang.Math.cos;
import static java.lang.Math.exp;
import static java.lang.Math.sqrt;

public class AnalyticOscillatorSystem implements TimeDrivenSimulationSystem {
  private final double k;
  private double systemTime;
  private Particle particle;
  private final double beta;


  /**
   * Set the initial conditions of the damped oscillator
   * @param mass the mass of the particle attached to the system
   * @param r the initial position of the particle
   * @param k the constant of the oscillator
   * @param gamma the damping factor
   */
  public AnalyticOscillatorSystem(final double mass, final double r, final double k, final double gamma) {
    this.k = k;
    beta = gamma / (2 * mass);

    // Create the particle with position, mass and initial velocity
    final double initialVx = -beta;

    this.particle = Particle.builder(r, 0)
            .mass(mass)
            .vx(initialVx)
            .build();
  }

  private double calculatePosition() {
    final double mass = particle.mass();

    // Used for visualization simplicity only
    final double aux1 = exp(-beta * systemTime);
    final double aux2 = sqrt((k / mass) - Math.pow(beta, 2));

    return aux1 * cos(aux2 * systemTime);
  }

  @Override
  public SystemData getSystemData() {
    return () -> {
      final Collection<Particle> particles = new HashSet<>();
      particles.add(particle);
      return particles;
    };
  }

  @Override
  public void evolveSystem(final double dt) {
    systemTime += dt;
    final double newX = calculatePosition();
    particle = particle.withX(newX);
  }


}
