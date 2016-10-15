package ar.edu.itba.ss.granularmedia.core.system.integration;

import ar.edu.itba.ss.granularmedia.interfaces.NeighboursFinder;
import ar.edu.itba.ss.granularmedia.interfaces.NumericIntegrationMethod;
import ar.edu.itba.ss.granularmedia.interfaces.SystemData;
import ar.edu.itba.ss.granularmedia.interfaces.TimeDrivenSimulationSystem;
import ar.edu.itba.ss.granularmedia.models.Particle;
import ar.edu.itba.ss.granularmedia.models.Vector2D;
import ar.edu.itba.ss.granularmedia.models.Wall;
import ar.edu.itba.ss.granularmedia.services.IOService;
import ar.edu.itba.ss.granularmedia.services.apis.Space2DMaths;
import ar.edu.itba.ss.granularmedia.services.gear.Gear5SystemData;
import ar.edu.itba.ss.granularmedia.services.gear.GearPredictorCorrector;
import ar.edu.itba.ss.granularmedia.services.neighboursfinders.CellIndexMethodImpl;

import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;

public class GearGranularMediaSystem implements TimeDrivenSimulationSystem {
  private static final double G = 9.80665;

  private final NumericIntegrationMethod<Gear5SystemData> integrationMethod;
  private final Gear5GranularMediaSystemData systemData;

  public GearGranularMediaSystem(final Collection<Particle> systemParticles,
                                 final Collection<Wall> systemWalls,
                                 final double kn, final double kt, final double L, final double W) {
    final Collection<Particle> updatedSystemParticles = new HashSet<>(systemParticles.size());
    systemParticles.forEach(particle -> {
      final Particle updatedParticle = particle.withForceY(-particle.mass() * G);
      updatedSystemParticles.add(updatedParticle);
    });

    this.systemData = new Gear5GranularMediaSystemData(updatedSystemParticles, systemWalls, kn, kt, L, W);
    this.integrationMethod = new GearPredictorCorrector<>();
  }

  @Override
  public SystemData getSystemData() {
    return systemData;
  }

  @Override
  public void evolveSystem(final double dt) {
    integrationMethod.evolveSystem(systemData, dt);
  }

  private static class Gear5GranularMediaSystemData extends Gear5SystemData {
    private static final double RC = 0;
    private static final boolean PERIODIC_LIMIT = false;

    private static final int NORMAL = 0;
    private static final int TANGENTIAL = 1;

    private static final int VELOCITY_DERIVED_ORDER = 1;

    private final double kn;
    private final double kt;
    private final double L;
    private final double W;
    private final double maxRadius;
    private final int M1;
    private final int M2;

    private final Collection<Wall> walls;
    private final NeighboursFinder neighboursFinder;

    private Map<Particle, Collection<Particle>> currentNeighbours;

    private Gear5GranularMediaSystemData(final Collection<Particle> particles,
                                         final Collection<Wall> walls,
                                         final double kn, final double kt,  final double L, final double W) {
      super(particles);
      this.kn = kn;
      this.kt = kt;
      this.L = L;
      this.W = W;
      this.walls = walls;
      this.neighboursFinder = new CellIndexMethodImpl();
      this.currentNeighbours = new HashMap<>(); // initialize so as not to be null
      this.maxRadius = initAndGetMaxRadio();

      final double condition1 = L / (RC + 2 * maxRadius); // M1 condition for cell index
      final double condition2 = W / (RC + 2 * maxRadius); // M2 condition for cell index

      if (condition1 == Math.floor(condition1)) { // In case condition1 is an "integer" value
        this.M1 = ((int)Math.floor(condition1)) - 1; // This is done to make sure M1 is strictly lesser than condition1
      } else {
        this.M1 = (int) Math.floor(condition1);
      }
      if (condition2 == Math.floor(condition2)) { // In case condition2 is an "integer" value
        this.M2 = ((int)Math.floor(condition2)) - 1; // This is done to make sure M2 is strictly lesser than condition2
      } else {
        this.M2 = (int) Math.floor(condition2);
      }
    }

    @Override
    protected Map<Integer, Vector2D> setInitialDerivativeValues(final Particle particle) {
      // it is considered that there is no interaction between any pair of particles
      final Map<Integer, Vector2D> initialDerivativeValues = new HashMap<>(sVectors());

      final Vector2D r0 = particle.r0();
      final Vector2D r1 = particle.r1();
      final Vector2D r2 = particle.r2();
      final Vector2D r3 = Space2DMaths.nullVector();
      final Vector2D r4 = Space2DMaths.nullVector();
      final Vector2D r5 = Space2DMaths.nullVector();

      initialDerivativeValues.put(0, r0);
      initialDerivativeValues.put(1, r1);
      initialDerivativeValues.put(2, r2);
      initialDerivativeValues.put(3, r3);
      initialDerivativeValues.put(4, r4);
      initialDerivativeValues.put(5, r5);

      return initialDerivativeValues;
    }

    @Override
    protected void preEvaluate() {
      // calculate neighbours with the system's particles updated with the predicted values
      this.currentNeighbours = neighboursFinder.run(predictedParticles(), L, W, M1, M2, RC, PERIODIC_LIMIT);
      super.preEvaluate();

    }

    @Override
    protected Vector2D getForceWithPredicted(final Particle particle) {
      // neighbours are supposed to be correctly updated
      final Vector2D totalParticlesForce = totalParticlesForce(particle, currentNeighbours.get(particle));
      final Vector2D totalWallsForce = totalWallsForce(particle);
      final Vector2D totalGravityForce = Vector2D.builder(0, - particle.mass() * G).build();

      return totalParticlesForce.add(totalWallsForce).add(totalGravityForce);
    }

    private double initAndGetMaxRadio() {
      double maxRadius = 0;
      for(final Particle particle : particles()){
        initParticle(particle);
        if(particle.radio() > maxRadius){
          maxRadius = particle.radio();
        }
      }
      return maxRadius;
    }

    // Particle's total force
    private Vector2D totalParticlesForce(final Particle particle, final Collection<Particle> neighbours) {
      Vector2D totalParticlesForce = Space2DMaths.nullVector();
      if (neighbours != null) {
        for (final Particle neighbour : neighbours) {
          final Vector2D neighbourForce = neighbourForce(particle, neighbour);
          totalParticlesForce = totalParticlesForce.add(neighbourForce);
        }
      }
      return totalParticlesForce;
    }

    private Vector2D neighbourForce(final Particle particle, final Particle neighbour) {
      final double superposition = Space2DMaths.superpositionBetween(particle, neighbour);

      final Vector2D[] normalAndTangentialVersors =
              Space2DMaths.normalAndTangentialVersors(particle.r0(), neighbour.r0());

      if (normalAndTangentialVersors == null) {
        // both particles are at the exactly same position => something is wrong...
        // Abort program
        IOService.exit(IOService.ExitStatus.PARTICLES_AT_SAME_POSITION, new Object[] {particle, neighbour});

        // should not reach here; written so as validators don't complain about possible null's access
        return Space2DMaths.nullVector();
      }

      final Vector2D normalVersor = normalAndTangentialVersors[NORMAL];
      final Vector2D tangentialVersor = normalAndTangentialVersors[TANGENTIAL];

      final Vector2D particlePredictedVelocity = getPredictedR(particle, VELOCITY_DERIVED_ORDER);
      final Vector2D neighbourPredictedVelocity = getPredictedR(neighbour, VELOCITY_DERIVED_ORDER);
      final Vector2D relativeVelocity = Space2DMaths.relativeVector(neighbourPredictedVelocity, particlePredictedVelocity);

      final Vector2D normalNeighbourForce = normalForce(superposition, normalVersor);
      final Vector2D tangentialNeighbourForce =
              tangentialForce(superposition, relativeVelocity, tangentialVersor);

      return normalNeighbourForce.add(tangentialNeighbourForce);
    }

    // Walls total force
    private Vector2D totalWallsForce(final Particle particle) {
      Vector2D totalWallsForce = Space2DMaths.nullVector();
      for (final Wall wall : walls) {
        final Vector2D wallForce = wallForce(particle, wall);
        totalWallsForce = totalWallsForce.add(wallForce);
      }

      return totalWallsForce;
    }

    private Vector2D wallForce(final Particle particle, final Wall wall) {
      final double superposition = Space2DMaths.superpositionBetween(particle, wall);
      if (superposition <= 0) { // not colliding => no force
        return Space2DMaths.nullVector();
      }

      final Vector2D[] normalAndTangentialVersors =
              Space2DMaths.normalAndTangentialVersors(particle, wall);

      if (normalAndTangentialVersors == null) {
        // both particles are at the exactly same position => something is wrong...
        // Abort program
        IOService.exit(IOService.ExitStatus.PARTICLES_AT_SAME_POSITION, new Object[] {particle, wall});

        // should not reach here; written so as validators don't complain about possible null's access
        return Space2DMaths.nullVector();
      }

      final Vector2D normalVersor = normalAndTangentialVersors[NORMAL];
      final Vector2D tangentialVersor = normalAndTangentialVersors[TANGENTIAL];

      final Vector2D relativeVelocity = getPredictedR(particle, VELOCITY_DERIVED_ORDER);

      final Vector2D normalForce = normalForce(superposition, normalVersor);
      final Vector2D tangentialForce = tangentialForce(superposition, relativeVelocity, tangentialVersor);

      return normalForce.add(tangentialForce);
    }

    // system's force calculation for both normal and tangential components

    private Vector2D normalForce(final double superposition, final Vector2D normalVersor) {
      final double normalNeighbourForceModule = - kn * superposition;
      return normalVersor.times(normalNeighbourForceModule);
    }

    private Vector2D tangentialForce(final double superposition,
                                     final Vector2D relativeVelocity,
                                     final Vector2D tangentialVersor) {
      final double tangentialRelativeVelocity = Space2DMaths.dotProduct(relativeVelocity, tangentialVersor);
      final double tangentialNeighbourForceModule = - kt * superposition *tangentialRelativeVelocity;

      return tangentialVersor.times(tangentialNeighbourForceModule);
    }
  }
}
