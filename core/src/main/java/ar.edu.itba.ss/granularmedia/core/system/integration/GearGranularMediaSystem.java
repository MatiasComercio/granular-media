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
import ar.edu.itba.ss.granularmedia.services.neighboursfinders.BruteForceMethodImpl;
<<<<<<< HEAD
import ar.edu.itba.ss.granularmedia.services.neighboursfinders.CellIndexMethodImpl;
=======
>>>>>>> 277d289... Added - our changes

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
<<<<<<< HEAD
                                 final double kn, final double kt, final double L, final double W) {
=======
                                 final double kn, final double kt) {
>>>>>>> 277d289... Added - our changes
    final Collection<Particle> updatedSystemParticles = new HashSet<>(systemParticles.size());
    systemParticles.forEach(particle -> {
      final Particle updatedParticle = particle.withForceY(-particle.mass() * G);
      updatedSystemParticles.add(updatedParticle);
    });

<<<<<<< HEAD
    this.systemData = new Gear5GranularMediaSystemData(updatedSystemParticles, systemWalls, kn, kt, L, W);
=======
    this.systemData = new Gear5GranularMediaSystemData(updatedSystemParticles, systemWalls, kn, kt);
>>>>>>> 277d289... Added - our changes
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
<<<<<<< HEAD
    private final double L;
    private final double W;
    private final double maxRadius;

=======
>>>>>>> 277d289... Added - our changes
    private final Collection<Wall> walls;
    private final NeighboursFinder neighboursFinder;

    private Map<Particle, Collection<Particle>> currentNeighbours;

    private Gear5GranularMediaSystemData(final Collection<Particle> particles,
                                         final Collection<Wall> walls,
<<<<<<< HEAD
                                         final double kn, final double kt,  final double L, final double W) {
      super(particles);
      this.kn = kn;
      this.kt = kt;
      this.L = L;
      this.W = W;
      this.walls = walls;
      this.neighboursFinder = new CellIndexMethodImpl();
      this.currentNeighbours = new HashMap<>(); // initialize so as not to be null
      this.maxRadius = getMaxRadius(particles);
      init();
    }

    // TODO: Along with init() this is going over all particles two times.
    private double getMaxRadius(Collection<Particle> particles) {
      double maxRadius = 0;
      for(Particle particle : particles){
        if(particle.radio() > maxRadius){
          maxRadius = particle.radio();
        }
      }
      return maxRadius;
    }

=======
                                         final double kn, final double kt) {
      super(particles);
      this.kn = kn;
      this.kt = kt;
      this.walls = walls;
      this.neighboursFinder = new BruteForceMethodImpl();
      this.currentNeighbours = new HashMap<>(); // initialize so as not to be null
      init();
    }

>>>>>>> 277d289... Added - our changes
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
<<<<<<< HEAD
      int M1, M2;
      final double condition1 = L / (RC + 2 * maxRadius); // M1 condition for cell index
      final double condition2 = W / (RC + 2 * maxRadius); // M2 condition for cell index

      if(condition1 == Math.floor(condition1)){ // In case condition1 is an "integer" value
        M1 = ((int)Math.floor(condition1)) - 1; // This is done to make sure M1 is strictly lesser than condition1
      } else{
        M1 = (int) Math.floor(condition1);
      }
      if(condition2 == Math.floor(condition2)){ // In case condition2 is an "integer" value
        M2 = ((int)Math.floor(condition2)) - 1; // This is done to make sure M2 is strictly lesser than condition2
      } else{
        M2 = (int) Math.floor(condition2);
      }
//      OLD CONDITION
//      final int M1 = ((int) Math.floor(L / (RC + 2 * maxRadius))) - 1; // TODO: M should be strictly < L / (RC + 2 * maxRadius)
//      final int M2 = ((int) Math.floor(W / (RC + 2 * maxRadius))) - 1;

      this.currentNeighbours = neighboursFinder.run(particles(), L, W, M1, M2, RC, PERIODIC_LIMIT); // +++xmagicnumber: M and L FIXME
      super.preEvaluate();

=======
      this.currentNeighbours = neighboursFinder.run(particles(), 0, 0, RC, PERIODIC_LIMIT); // +++xmagicnumber: M and L FIXME
      super.preEvaluate();
>>>>>>> 277d289... Added - our changes
    }

    @Override
    protected Vector2D getForceWithPredicted(final Particle particle) {
      // neighbours are supposed to be correctly updated
      final Vector2D totalParticlesForce = totalParticlesForce(particle, currentNeighbours.get(particle));
      final Vector2D totalWallsForce = totalWallsForce(particle);
      final Vector2D totalGravityForce = Vector2D.builder(0, - particle.mass() * G).build();

      return totalParticlesForce.add(totalWallsForce).add(totalGravityForce);
    }

    // Particle's total force
    private Vector2D totalParticlesForce(final Particle particle, final Collection<Particle> neighbours) {
      Vector2D totalParticlesForce = Space2DMaths.nullVector();
      for (final Particle neighbour : neighbours) {
        final Vector2D neighbourForce = neighbourForce(particle, neighbour);
        totalParticlesForce = totalParticlesForce.add(neighbourForce);
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
<<<<<<< HEAD
      final Vector2D relativeVelocity = Space2DMaths.relativeVector(neighbourPredictedVelocity, particlePredictedVelocity);
=======
      final Vector2D relativeVelocity = Space2DMaths.relativeVector(particlePredictedVelocity, neighbourPredictedVelocity);
>>>>>>> 277d289... Added - our changes

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
