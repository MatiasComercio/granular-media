package ar.edu.itba.ss.granularmedia.services.gear;

import ar.edu.itba.ss.granularmedia.interfaces.TimeDrivenSimulationSystem;
import ar.edu.itba.ss.granularmedia.models.Particle;
import ar.edu.itba.ss.granularmedia.services.gear.oscillator.AnalyticOscillatorSystem;
import ar.edu.itba.ss.granularmedia.services.gear.oscillator.GearOscillatorSystem;
import org.junit.Before;
import org.junit.Test;

public class GearPredictorCorrectorTest {
  private static final double mass = 70;
  private static final double r = 1;
  private static final double k = 10000;
  private static final double gamma = 100;
  private static final double tf = 5;
  private static final double dt = 0.001;

  private TimeDrivenSimulationSystem analyticOscillator;
  private TimeDrivenSimulationSystem gearOscillator;


  @Before
  public void setEnvironment() {
    this.gearOscillator = new GearOscillatorSystem(mass, r, k, gamma);
    this.analyticOscillator = new AnalyticOscillatorSystem(mass, r, k, gamma);
  }

  @Test
  public void evolveSystem() {
    for(double systemTime = 0; systemTime < tf; systemTime += dt) {
      analyticOscillator.evolveSystem(dt);
      gearOscillator.evolveSystem(dt);

      final Particle aParticle = analyticOscillator.getSystemData().particles().iterator().next();
      final Particle gParticle = gearOscillator.getSystemData().particles().iterator().next();

      assert Math.abs(aParticle.x() - gParticle.x()) < 1e-7;
    }
  }
}
