package ar.edu.itba.ss.granularmedia.core.system;

import ar.edu.itba.ss.granularmedia.interfaces.NeighboursFinder;
import ar.edu.itba.ss.granularmedia.models.Particle;
import ar.edu.itba.ss.granularmedia.models.StaticData;
import ar.edu.itba.ss.granularmedia.models.Vector2D;
import ar.edu.itba.ss.granularmedia.models.Wall;
import ar.edu.itba.ss.granularmedia.services.IOService;
import ar.edu.itba.ss.granularmedia.services.RandomService;
import ar.edu.itba.ss.granularmedia.services.apis.Space2DMaths;
import ar.edu.itba.ss.granularmedia.services.gear.Gear5SystemData;
import ar.edu.itba.ss.granularmedia.services.neighboursfinders.CellIndexMethodImpl;

import java.util.*;

public class Gear5GranularMediaSystemData extends Gear5SystemData {
  private static final double G = 9.80665;
  private static final double RC = 0;
  private static final boolean PERIODIC_LIMIT = false;

  private static final int NORMAL = 0;
  private static final int TANGENTIAL = 1;

  private static final int VELOCITY_DERIVED_ORDER = 1;

  private static final int RESPAWN_MAX_TRIES = 5;
  private static final double ZERO = 0; // +++xcheck: repeated variable in other clases

  private final double kn;
  private final double kt;

  private final Collection<Wall> walls;
  private final NeighboursFinder neighboursFinder;
  private final Deque<Particle> respawnQueue;
  private final RespawnArea respawnArea;

  private Map<Particle, Collection<Particle>> currentNeighbours;

  public Gear5GranularMediaSystemData(final Collection<Particle> particles,
                                      final Collection<Wall> walls,
                                      final StaticData staticData) {
    super(particles);
    this.kn = staticData.kn();
    this.kt = staticData.kt();

    this.walls = Collections.unmodifiableCollection(walls);
//      this.neighboursFinder = new BruteForceMethodImpl(PERIODIC_LIMIT, RC); // +++xdebug
    this.currentNeighbours = new HashMap<>(); // initialize so as not to be null
    this.respawnQueue = new LinkedList<>();

    final double totalSystemLength = staticData.totalSystemLength();
    final double width = staticData.width();

    final double maxRadius = initAndGetMaxRadio();

    final double respawnMinX = ZERO;
    final double respawnMaxX = respawnMinX + width;

    final double respawnMinY = staticData.fallLength() + staticData.length();
    final double respawnMaxY = respawnMinY + 2 * maxRadius;

    this.respawnArea = new RespawnArea(respawnMinX, respawnMaxX, respawnMinY, respawnMaxY, maxRadius);

    final double condition1 = totalSystemLength / (RC + 2 * maxRadius); // M1 condition for cell index
    final double condition2 = width / (RC + 2 * maxRadius); // M2 condition for cell index

    int m1, m2;
    if (condition1 == Math.floor(condition1)) { // In case condition1 is an "integer" value
      m1 = ((int)Math.floor(condition1)) - 1; // This is done to make sure M1 is strictly lesser than condition1
    } else {
      m1 = (int) Math.floor(condition1);
    }
    if (condition2 == Math.floor(condition2)) { // In case condition2 is an "integer" value
      m2 = ((int)Math.floor(condition2)) - 1; // This is done to make sure M2 is strictly lesser than condition2
    } else {
      m2 = (int) Math.floor(condition2);
    }

    this.neighboursFinder = new CellIndexMethodImpl(totalSystemLength, width, m1, m2, RC, PERIODIC_LIMIT);
  }

  public Collection<Wall> walls() {
    return walls;
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
    // reset maxPressure
    Particle.setMaxPressure(0);

    // calculate neighbours with the system's particles updated with the predicted values
    this.currentNeighbours = neighboursFinder.run(predictedParticles());
    super.preEvaluate();
  }

  @Override
  public void predicted(final Particle predictedParticle) {
    // it is assumed that if the predicted particle.y() is < ZERO => the particle will be out soon =>
    // => we remove that particle before evaluation for simplification on neighbours finder method usage
    removeIfOut(predictedParticle);

    super.predicted(predictedParticle);
  }

  private void removeIfOut(final Particle particle) {
    if(particle.y() < ZERO){
      respawnQueue.add(particle);
      neighboursFinder.avoid(particle);
      removeWhenFinish(particle);
    }
  }

  @Override
  public void fixed(final Particle particle) {
    removeIfOut(particle);
    respawnArea.update(particle);

    super.fixed(particle);
  }

  @SuppressWarnings("SpellCheckingInspection")
  @Override
  protected void postFix() {
    super.postFix();

    Iterator<Particle> iterator = respawnQueue.iterator();

    while (respawnArea.hasNextCell() && iterator.hasNext()) {
      final Particle particle = iterator.next();
      final Particle respawned = respawnArea.respawn(particle);
      spawnParticle(respawned);
      iterator.remove();
    }
  }

  private void spawnParticle(final Particle particle) {
    this.particles().add(particle);
    this.predictedRs().put(particle, new HashMap<>(sVectors()));
    this.currentRs().put(particle, setInitialDerivativeValues(particle));
  }

  @Override
  protected Vector2D getForceWithPredicted(final Particle particle) {
    particle.normalForce(0); // reset forces for this iteration

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
    final Vector2D tangentialNeighbourForce = tangentialForce(superposition, relativeVelocity, tangentialVersor);

    particle.increaseNormalForce(normalNeighbourForce.norm2());

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

    particle.increaseNormalForce(normalForce.norm2()); // increase normal force

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

  private static class RespawnArea {
    private final Deque<Cell> emptyCells;
    private final Map<Particle, Cell> fullCells;

    private RespawnArea(final double respawnMinX, final double respawnMaxX,
                        final double respawnMinY, final double respawnMaxY,
                        final double maxRadius) {

      this.emptyCells = new LinkedList<>();
      this.fullCells = new HashMap<>();

      final double cellSize = (1.1 * maxRadius);
      final double yCells = (respawnMaxY + respawnMinY) / 2;

      for (double i = respawnMinX; i < respawnMaxX ; i += cellSize) {
        final double x = (i + cellSize) / 2;
        emptyCells.add(new Cell(x, yCells));
      }
    }

    private boolean hasNextCell() {
      return !emptyCells.isEmpty();
    }

    private Particle respawn(final Particle particle) {
      final Cell cell = emptyCells.poll();
      final Particle respawnedParticle = Particle.builder(cell.x, cell.y).id(particle.id())
              .radio(particle.radio()).mass(particle.mass()).forceY(-particle.mass() * G).type(particle.type())
              .build();

      fullCells.put(respawnedParticle, cell);
      return respawnedParticle;
    }

    private void update(final Particle particle) {
      final Cell cell = fullCells.get(particle);
      if (cell == null) {
        return;
      }

      // particle was respawned recently
      if (particle.y() < cell.y) { // out of respawn area
        fullCells.remove(particle);
      }
    }

    private static class Cell {
      private final double x;
      private final double y;

      private Cell(final double x, final double y) {
        this.x = x;
        this.y = y;
      }
    }
  }
}
