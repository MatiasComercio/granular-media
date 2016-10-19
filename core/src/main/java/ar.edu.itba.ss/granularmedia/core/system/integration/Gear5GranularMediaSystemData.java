package ar.edu.itba.ss.granularmedia.core.system.integration;

import ar.edu.itba.ss.granularmedia.interfaces.NeighboursFinder;
import ar.edu.itba.ss.granularmedia.models.Particle;
import ar.edu.itba.ss.granularmedia.models.StaticData;
import ar.edu.itba.ss.granularmedia.models.Vector2D;
import ar.edu.itba.ss.granularmedia.models.Wall;
import ar.edu.itba.ss.granularmedia.services.IOService;
import ar.edu.itba.ss.granularmedia.services.apis.Space2DMaths;
import ar.edu.itba.ss.granularmedia.services.gear.Gear5SystemData;
import ar.edu.itba.ss.granularmedia.services.neighboursfinders.BruteForceMethodImpl;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;

public class Gear5GranularMediaSystemData extends Gear5SystemData {
  @SuppressWarnings("unused")
  private static final Logger LOGGER = LoggerFactory.getLogger(Gear5GranularMediaSystemData.class);

  private static final double G = 9.80665;
  private static final double RC = 0;
  private static final boolean PERIODIC_LIMIT = false;

  private static final int NORMAL = 0;
  private static final int TANGENTIAL = 1;

  private static final int VELOCITY_DERIVED_ORDER = 1;

  private static final double ZERO = 0;

  private final double kn;
  private final double kt;

  private final Collection<Wall> walls;
  private final NeighboursFinder neighboursFinder;
  private final Deque<Particle> respawnQueue;
  private final RespawnArea respawnArea;
  private final double fallLength;

  private Map<Particle, Collection<Particle>> currentNeighbours;
  private double kineticEnergy;
  private long nParticlesFlowed;
  private long nParticlesJustFlowed;

  /* package-private */ Gear5GranularMediaSystemData(final Collection<Particle> particles,
                               final Collection<Wall> walls,
                               final StaticData staticData) {
    super(particles);
    this.kn = staticData.kn();
    this.kt = staticData.kt();

    this.walls = Collections.unmodifiableCollection(walls);
    this.currentNeighbours = new HashMap<>(); // initialize so as not to be null
    this.respawnQueue = new LinkedList<>();
    this.fallLength = staticData.fallLength();

    final double width = staticData.width();

    final double maxRadius = initAndGetMaxRadio();

    final double respawnMinX = ZERO;
    final double respawnMaxX = respawnMinX + width;

    this.respawnArea = new RespawnArea(respawnMinX, respawnMaxX,
            staticData.respawnMinY(), staticData.respawnMaxY(), maxRadius);
    this.neighboursFinder = new BruteForceMethodImpl(PERIODIC_LIMIT, RC);
  }

  public Collection<Wall> walls() {
    return walls;
  }

  public double kineticEnergy() {
    return kineticEnergy;
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
  protected Vector2D getForceWithPredicted(final Particle particle) {
    particle.normalForce(0); // reset forces for this iteration

    // neighbours are supposed to be correctly updated
    final Vector2D totalParticlesForce = totalParticlesForce(particle, currentNeighbours.get(particle));
    final Vector2D totalWallsForce = totalWallsForce(particle);
    final Vector2D totalGravityForce = Vector2D.builder(0, - particle.mass() * G).build();

    return totalParticlesForce.add(totalWallsForce).add(totalGravityForce);
  }

  @Override
  protected void prePredict() {
    // reset kinetic energy
    kineticEnergy = 0;

    // reset maxPressure
    Particle.setMaxPressure(0);

    // reset nParticlesJustFlowed
    nParticlesJustFlowed = 0;
  }

  @Override
  protected void predicted(final Particle predictedParticle) {
    // it is assumed that if the predicted particle.y() is < ZERO => the particle will be out soon =>
    // => we remove that particle before evaluation for simplification on neighbours finder method usage
    removeIfOut(predictedParticle);

    super.predicted(predictedParticle);
  }

  @Override
  protected void preEvaluate() {
    // calculate neighbours with the system's particles updated with the predicted values
    this.currentNeighbours = neighboursFinder.run(predictedParticles());
    super.preEvaluate();
  }

  public long nParticlesFlowed() {
    return nParticlesFlowed;
  }

  public long nParticlesJustFlowed() {
    return nParticlesJustFlowed;
  }

  @Override
  public void fixed(final Particle particle) {
    if (flowedOut(particle)) {
      nParticlesJustFlowed ++;
      nParticlesFlowed ++;
    }
    if (!removeIfOut(particle)) {
      kineticEnergy += particle.kineticEnergy();
    }
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

  /**
   *
   * @param particle -
   * @return true if removed; false otherwise
   */
  private boolean removeIfOut(final Particle particle) {
    if(particle.y() < ZERO){
      respawnQueue.add(particle);
      removeWhenFinish(particle);
      return true;
    }
    return false;
  }

  private boolean flowedOut(final Particle particle) {
    // professor told us to use only the particle's center point, not including its radio
    if (particle.y() < fallLength && !particle.hasFlowedOut()) {
      particle.hasFlowedOut(true);
      return true;
    }
    return false;
  }

  private void spawnParticle(final Particle particle) {
    this.particles().add(particle);
    this.predictedRs().put(particle, new HashMap<>(sVectors()));
    this.currentRs().put(particle, setInitialDerivativeValues(particle));
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
    if (superposition < 0) {
      return Space2DMaths.nullVector();
    }

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
    private final Map<Particle, Collection<Cell>> fullCellsMap;
    private final int[] particlesOnCell;
    private final double cellSize;
    private final double yCells;
    private final double minXPos;
    private final double maxXPos;

    private final double respawnMinX;
    private final double respawnMaxX;

    private final HashMap<Integer, Cell> respawnCellsMap;

    private RespawnArea(final double respawnMinX, final double respawnMaxX,
                        final double respawnMinY, final double respawnMaxY,
                        final double maxRadius) {

      this.emptyCells = new LinkedList<>();
      this.respawnCellsMap = new HashMap<>();
      this.fullCellsMap = new HashMap<>();

      final double diameter = 2 * maxRadius;
      this.cellSize = (1.1 * diameter);
      this.yCells = (respawnMaxY + respawnMinY) / 2;
      this.minXPos = respawnMinX + cellSize/2;
      this.maxXPos = respawnMaxX - cellSize/2;

      this.respawnMinX = respawnMinX;
      this.respawnMaxX = respawnMaxX;

      final int nCells = (int) ((maxXPos - minXPos)/cellSize) + 1;
      this.particlesOnCell = new int[nCells];
      int index = 0;
      for (double i = minXPos; i < maxXPos ; i += cellSize, index++) {
        final Cell cell = new Cell(i, yCells, index);
        emptyCells.add(cell);
        respawnCellsMap.put(index, cell);
      }
    }

    private boolean hasNextCell() {
      return !emptyCells.isEmpty();
    }

    private Particle respawn(final Particle particle) {
      final Cell cell = emptyCells.poll();
      final Particle respawnedParticle = particle.respawn(cell.x, cell.y, 0, - particle.mass() * G);

      final Set<Cell> cells = new HashSet<>();
      cells.add(cell);
      fullCellsMap.put(respawnedParticle, cells);
      particlesOnCell[cell.index] ++;
      return respawnedParticle;
    }

    private void update(final Particle particle) {
      final Collection<Cell> cells = fullCellsMap.get(particle);
      // if particle does not have taken cells, return
      if (cells == null) {
        return;
      }

      // let's update each taken cell with current position

      // first, remove previous cells
      for (final Cell cell : cells) {
        fullCellsMap.remove(particle);
        particlesOnCell[cell.index] --;
        if (particlesOnCell[cell.index] == 0) { // no more particles on this cell => empty cell
          emptyCells.add(cell);
        }
      }


      // particle is down of the respawn area, then finished update
      if (particle.y() + particle.radio() < yCells - cellSize) {
        return;
      }

      // particle is occupying some respawn cells (at most 2)

      // take cells for that particle so as to avoid making the system crash
      final Collection<Cell> takenCells = getTakenCells(particle);
      // occupied
      fullCellsMap.put(particle, takenCells);
      for (final Cell takenCell : takenCells) {
        emptyCells.remove(takenCell); // not available any more
        particlesOnCell[takenCell.index] ++;
      }
    }

    private Collection<Cell> getTakenCells(final Particle particle) {
      final Collection<Cell> takenCells = new HashSet<>();
      final double pX = particle.x();
      int index = 0;
      for (double i = respawnMinX ; i < respawnMaxX ; i += cellSize, index ++) {
        if (i > pX) {
          // could be because we are considering the neighbour cell too
          addIfNotNull(takenCells, respawnCellsMap.get(index - 1));
          addIfNotNull(takenCells, respawnCellsMap.get(index));
          return takenCells;
        }
      }
      // should never reach here; if so, particle is out of the map
      return takenCells;
    }

    private void addIfNotNull(final Collection<Cell> takenCells, final Cell cell) {
      if (cell != null) {
        takenCells.add(cell);
      }
    }

    private static class Cell {
      private final double x;
      private final double y;
      private final int index;

      private Cell(final double x, final double y, final int index) {
        this.x = x;
        this.y = y;
        this.index = index;
      }
    }
  }
}
