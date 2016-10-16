package ar.edu.itba.ss.granularmedia.core.system.integration;

import ar.edu.itba.ss.granularmedia.core.system.Gear5GranularMediaSystemData;
import ar.edu.itba.ss.granularmedia.interfaces.NumericIntegrationMethod;
import ar.edu.itba.ss.granularmedia.interfaces.TimeDrivenSimulationSystem;
import ar.edu.itba.ss.granularmedia.models.Particle;
import ar.edu.itba.ss.granularmedia.models.StaticData;
import ar.edu.itba.ss.granularmedia.models.Wall;
import ar.edu.itba.ss.granularmedia.services.gear.Gear5SystemData;
import ar.edu.itba.ss.granularmedia.services.gear.GearPredictorCorrector;

import java.util.*;

public class GearGranularMediaSystem
        implements TimeDrivenSimulationSystem<Gear5GranularMediaSystemData> {
  private static final double G = 9.80665;

  private final NumericIntegrationMethod<Gear5SystemData> integrationMethod;
  private final Gear5GranularMediaSystemData systemData;

  public GearGranularMediaSystem(final Collection<Particle> systemParticles,
                                 final Collection<Wall> systemWalls, final StaticData staticData) {
    final Collection<Particle> updatedSystemParticles = new HashSet<>(systemParticles.size());
    systemParticles.forEach(particle -> {
      final Particle updatedParticle = particle.withForceY(-particle.mass() * G);
      updatedSystemParticles.add(updatedParticle);
    });

    // Notice length is the whole system's length (silo's length + fallLength + respawnLength) and not
    // simply the silo's length
    this.systemData = new Gear5GranularMediaSystemData(updatedSystemParticles, systemWalls, staticData);
    this.integrationMethod = new GearPredictorCorrector<>();
  }

  @Override
  public Gear5GranularMediaSystemData getSystemData() {
    return systemData;
  }

  @Override
  public void evolveSystem(final double dt) {
    integrationMethod.evolveSystem(systemData, dt);
  }
}
