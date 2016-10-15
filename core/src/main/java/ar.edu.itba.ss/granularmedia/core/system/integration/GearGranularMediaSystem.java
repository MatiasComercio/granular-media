package ar.edu.itba.ss.granularmedia.core.system.integration;

import ar.edu.itba.ss.granularmedia.interfaces.NeighboursFinder;
import ar.edu.itba.ss.granularmedia.interfaces.NumericIntegrationMethod;
import ar.edu.itba.ss.granularmedia.interfaces.SystemData;
import ar.edu.itba.ss.granularmedia.interfaces.TimeDrivenSimulationSystem;
import ar.edu.itba.ss.granularmedia.models.Particle;
import ar.edu.itba.ss.granularmedia.models.Vector2D;
import ar.edu.itba.ss.granularmedia.models.Wall;
import ar.edu.itba.ss.granularmedia.services.IOService;
import ar.edu.itba.ss.granularmedia.services.RandomService;
import ar.edu.itba.ss.granularmedia.services.apis.Space2DMaths;
import ar.edu.itba.ss.granularmedia.services.gear.Gear5SystemData;
import ar.edu.itba.ss.granularmedia.services.gear.GearPredictorCorrector;
import ar.edu.itba.ss.granularmedia.services.neighboursfinders.BruteForceMethodImpl;
import ar.edu.itba.ss.granularmedia.services.neighboursfinders.CellIndexMethodImpl;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;

public class GearGranularMediaSystem implements TimeDrivenSimulationSystem {
    private static final Logger LOGGER = LoggerFactory.getLogger(Gear5GranularMediaSystemData.class);
  private static final double G = 9.80665;

  private final NumericIntegrationMethod<Gear5SystemData> integrationMethod;
  private final Gear5GranularMediaSystemData systemData;

  public GearGranularMediaSystem(final Collection<Particle> systemParticles,
                                 final Collection<Wall> systemWalls, final double kn, final double kt,
                                 final double L, final double W, final double fallLength, final double respawnLength) {
    final Collection<Particle> updatedSystemParticles = new HashSet<>(systemParticles.size());
    systemParticles.forEach(particle -> {
      final Particle updatedParticle = particle.withForceY(-particle.mass() * G);
      updatedSystemParticles.add(updatedParticle);
    });

    // Notice L is the whole system's length (silo's length + fallLength + respawnLength) and not
    // simply the silo's length
    this.systemData = new Gear5GranularMediaSystemData(updatedSystemParticles, systemWalls, kn, kt, L, W, fallLength, respawnLength);
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
    private final double fallLength;
    private final double respawnLength;
    private final double maxRadius;

    private final Collection<Wall> walls;
    private final NeighboursFinder neighboursFinder;
    private final Deque<Particle> respawnQueue;

    private Map<Particle, Collection<Particle>> currentNeighbours;

    private Gear5GranularMediaSystemData(final Collection<Particle> particles,
                                         final Collection<Wall> walls, final double kn, final double kt,
                                         final double L, final double W, final double fallLength, final double respawnLength) {
      super(particles);
      this.kn = kn;
      this.kt = kt;
      this.L = L;
      this.W = W;
      this.fallLength = fallLength;
      this.respawnLength = respawnLength;
      this.walls = walls;
      this.neighboursFinder = new CellIndexMethodImpl();
      this.currentNeighbours = new HashMap<>(); // initialize so as not to be null
      this.respawnQueue = new LinkedList<>();
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

      this.currentNeighbours = neighboursFinder.run(particles(), L, W, M1, M2, RC, PERIODIC_LIMIT); // +++xmagicnumber: M and L FIXME
      super.preEvaluate();

    }

    @Override
    protected void postFix() {
      final int MAX_TRIES = 5;

      // TODO: Avoid going over particles 2 times: Add particles in the Fix() method
      for(Particle particle: particles()){
        if(particle.y() < fallLength){
          respawnQueue.add(particle);
        }
      }

      for(int i=0; i<respawnQueue.size(); i++){
        // L is currently the system's length, hence (L-fallLength-respawnLength) is the length of the silo
        respawn(L-fallLength-respawnLength, W, fallLength, respawnLength, respawnQueue.poll(), MAX_TRIES);
      }

      super.postFix();
    }

    private void respawn(double L, double W, double fallLength, double respawnLength, Particle particle, int maxTries) {
      deleteParticle(particle);
      Particle updatedParticle = updateParticle(L, W, fallLength, respawnLength, particle, maxTries);
      if(updatedParticle != null){ // if it's null, then maxTries were reached trying to update the particle
          spawnParticle(updatedParticle);
      }
    }

    private void deleteParticle(Particle particle) {
      particles().remove(particle);
      predictedRs().remove(particle);
      currentRs().remove(particle);
    }

    private Particle updateParticle(double L, double W, double fallLength, double respawnLength,
                                    Particle particle, int maxTries) {
      final double minX = 0;
      final double maxX = W;
      final double minY = fallLength + L;
      final double maxY = fallLength + L + respawnLength;
      double pX;
      double pY;
      int tries = 0;
      do{
          pX = RandomService.randomDouble(minX, maxX);
          pY = RandomService.randomDouble(minY, maxY);
          tries++;

      } while(isColliding(pX, pY, particle.radio()) && tries < maxTries);

      if(tries >= maxTries){
          respawnQueue.add(particle); // Queue the particle to be updated on next system evolve
          return null;
      }
      return Particle.builder(pX, pY).id(particle.id()).radio(particle.radio()).mass(particle.mass()).forceY(-particle.mass() * G).build();
    }

      private boolean isColliding(double pX, double pY, double radio){
          double distanceX, distanceY, distance;
          // TODO: use a single cell index iteration? (Although it may be less efficient
          // due to the overhead of creating the structure)
          for(Particle particle : particles()){
            distanceX = particle.x() - pX;
            distanceY = particle.y() - pY;
            distance = Math.hypot(distanceX, distanceY);
            if(distance <= (particle.radio() + radio)){
                LOGGER.debug("Collision detected when respawning particles");
                return true;
            }
          }
          return false;
      }

      private void spawnParticle(Particle particle) {
      this.particles().add(particle);
      this.predictedRs().put(particle, new HashMap<>(sVectors()));
      this.currentRs().put(particle, setInitialDerivativeValues(particle));
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
