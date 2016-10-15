package ar.edu.itba.ss.granularmedia.core.system;

import ar.edu.itba.ss.granularmedia.core.helpers.OutputSerializerHelper;
import ar.edu.itba.ss.granularmedia.core.system.integration.GearGranularMediaSystem;
import ar.edu.itba.ss.granularmedia.interfaces.MainProgram;
import ar.edu.itba.ss.granularmedia.interfaces.TimeDrivenSimulationSystem;
import ar.edu.itba.ss.granularmedia.models.Particle;
import ar.edu.itba.ss.granularmedia.models.StaticData;
import ar.edu.itba.ss.granularmedia.models.Wall;
import ar.edu.itba.ss.granularmedia.services.IOService;
import ar.edu.itba.ss.granularmedia.services.factories.ParticleFactory;
import ar.edu.itba.ss.granularmedia.services.RandomService;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.nio.file.Path;
import java.util.Collection;
import java.util.HashSet;
import java.util.stream.Collectors;

import static ar.edu.itba.ss.granularmedia.services.IOService.ExitStatus.COULD_NOT_OPEN_OUTPUT_FILE;
import static ar.edu.itba.ss.granularmedia.services.IOService.ExitStatus.VALIDATION_FAILED;

public class GranularMediaSystemProgram implements MainProgram {
  private static final Logger LOGGER = LoggerFactory.getLogger(GranularMediaSystemProgram.class);

  // system constants
  private static final int N_PARTICLES = 4;
  private static final double LENGTH = 5; // Silo's length
  private static final double WIDTH = 3; // Silo's width
  private static final double FALL_LENGTH = 1.5; // Length of the area where particles fall out of the silo
  private static final double RESPAWN_LENGTH = 1.5; // Length of the area where particles respawn
  private static final double SYSTEM_LENGTH = RESPAWN_LENGTH + LENGTH + FALL_LENGTH;
  private static final double DIAMETER_OPENING = 1.5; // Length of the opening
  private static final double MASS = 0.01;
  private static final double KN = 10e5;
  private static final double KT = 2 * KN;
  private static final double MIN_DIAMETER = DIAMETER_OPENING /7;
  private static final double MAX_DIAMETER = DIAMETER_OPENING /5;
  private static final double SIMULATION_TIME = 2.5;
  private static final double DEFAULT_DELTA_1 = .1 * Math.sqrt(MASS/KN);
  private static final double DELTA_1 = 1e-6;
  private static final double DELTA_2 = 0.001;
  // condition constants
  private static final boolean OVERLAP_ALLOWED = false;
  private static final int MAX_OVERLAP_TRIES = 100;

  // non-magic number constants
  private static final double ZERO = 0;

  // file constants
  private static final String OVITO_FILE_EXTENSION = ".xyz";
  private static final String DEFAULT_OUTPUT_FOLDER = "output";
  private static final String DEFAULT_OVITO_FILE_NAME = "ovito";
  private static final double MS_TO_S = 1/1000.0;
  private static final double DELTA_LOG = 0.5;

  @Override
  public void run(final String[] args) {
    // system's particles
    final Collection<Particle> systemParticles = initializeSystemParticles();

    // system's walls
    final Collection<Wall> systemWalls = initializeSystemWalls(SYSTEM_LENGTH, WIDTH, DIAMETER_OPENING); // +++xmagicnumber

    final TimeDrivenSimulationSystem granularMediaSystem =
            new GearGranularMediaSystem(systemParticles, systemWalls, KN, KT, SYSTEM_LENGTH, WIDTH, FALL_LENGTH, RESPAWN_LENGTH);

    // static data
    final StaticData staticData =
            StaticData.builder(N_PARTICLES, WIDTH, SYSTEM_LENGTH, 0, SIMULATION_TIME).build(); // +++xmagicnumber

    // helper to write ovito file
    final OutputSerializerHelper outputSerializerHelper = new OutputSerializerHelper(staticData);

    // default delta time
    final double dt = Math.min(DEFAULT_DELTA_1, DELTA_1);
    LOGGER.info("Chosen dt: {}s", dt);

    // simulation itself
    LOGGER.info("Starting simulation...");
    startSimulation(granularMediaSystem, dt, staticData.simulationTime(), DELTA_2, outputSerializerHelper);
    LOGGER.info("[FINISHED]");
  }

  // private
  private void startSimulation(final TimeDrivenSimulationSystem granularMediaSystem,
                               final double dt, final double simulationTime, final double delta2,
                               final OutputSerializerHelper outputSerializerHelper) {
    final Path pathToOvitoFile = createOvito(DEFAULT_OUTPUT_FOLDER, DEFAULT_OVITO_FILE_NAME);

    final double startTime = System.currentTimeMillis();

    long step = 0;
    long logStep = 0;
    double currentTime = 0;
    while (currentTime < simulationTime) {
      // print system after printStepGap dt units
      if (currentTime >= (delta2 * step)) {
        appendToOvito(pathToOvitoFile,
                granularMediaSystem.getSystemData().particles(),
                step++, outputSerializerHelper);
        if (currentTime >= (DELTA_LOG * logStep)) {
          LOGGER.debug("Current time: {} ; Simulation Time: {} ; Step: {}", currentTime, simulationTime, step);
          logStep ++;
        }
      }

      // evolve system
      granularMediaSystem.evolveSystem(dt);

      // advance time and count the current step
      currentTime += dt;
    }

    IOService.closeOutputFile(pathToOvitoFile);

    final double endTime = System.currentTimeMillis();
    final double simulationDuration = endTime - startTime;
    LOGGER.info("Total simulation time: {} s", simulationDuration * MS_TO_S);
  }

  private Collection<Particle> initializeSystemParticles() {
    final ParticleFactory particleFactory = ParticleFactory.getInstance();

    if (!validParametersRange(LENGTH, WIDTH, DIAMETER_OPENING)) {
      IOService.exit(VALIDATION_FAILED, "length > width > diameterOpening");
    }

    final double[] radios = new double[N_PARTICLES];
    double diameter;
    for (int i = 0  ; i < N_PARTICLES ; i++) {
      diameter = randomDiameter();
      radios[i] = diameter/2;
    }

    final Particle leftBottomParticle = particleFactory.create(ZERO, FALL_LENGTH);
    final Particle rightTopParticle = particleFactory.create(WIDTH + ZERO, FALL_LENGTH + LENGTH + ZERO);

    return particleFactory.randomPoints(
            leftBottomParticle,
            rightTopParticle,
            radios, MASS,
            OVERLAP_ALLOWED,
            MAX_OVERLAP_TRIES);
  }

  private Collection<Wall> initializeSystemWalls(final double length,
                                                 final double width,
                                                 final double diameterOpening) {
    final Collection<Wall> systemWalls = new HashSet<>();
    final Wall leftVerticalWall = Wall.builder(ZERO, ZERO, ZERO, length).build();
    final Wall rightVerticalWall = Wall.builder(width, ZERO, width, length).build();

    final double horizontalWallWidth = (width-diameterOpening) / 2;

    @SuppressWarnings("UnnecessaryLocalVariable")
    final double xFromLeftHorizontalWall = ZERO;
    final double xToLeftHorizontalWall = xFromLeftHorizontalWall + horizontalWallWidth;

    final double xFromRightHorizontalWall = (width+diameterOpening) / 2;
    final double xToRightHorizontalWall = xFromRightHorizontalWall + horizontalWallWidth;

    final Wall leftBottomHorizontalWall =
            Wall.builder(xFromLeftHorizontalWall, FALL_LENGTH, xToLeftHorizontalWall, FALL_LENGTH).build();
    final Wall rightBottomHorizontalWall =
            Wall.builder(xFromRightHorizontalWall, FALL_LENGTH, xToRightHorizontalWall, FALL_LENGTH).build();

    systemWalls.add(leftVerticalWall);
    systemWalls.add(rightVerticalWall);
    systemWalls.add(leftBottomHorizontalWall);
    systemWalls.add(rightBottomHorizontalWall);

    return systemWalls;
  }


  /**
   *
   * @param defaultOutputFolder folder to save Ovito's file
   * @param defaultOvitoFileName Ovito's file name without extension
   * @return path to the created Ovito's file
   */
  private Path createOvito(final String defaultOutputFolder,
                           final String defaultOvitoFileName) {
    final String ovitoFile = defaultOvitoFileName + OVITO_FILE_EXTENSION;
    final Path pathToOvitoFile = IOService.createFile(defaultOutputFolder, ovitoFile);
    if (!IOService.openOutputFile(pathToOvitoFile, true)) {
      IOService.exit(COULD_NOT_OPEN_OUTPUT_FILE, pathToOvitoFile);
    }
    // only reach here if could open file
    return pathToOvitoFile;
  }

  private void appendToOvito(final Path ovitoFilePath,
                             final Collection<Particle> particleSet,
                             final long iteration,
                             final OutputSerializerHelper outputSerializerHelper) {
    final String ovitoOutputData = outputSerializerHelper.ovitoOutput(particleSet, iteration);
    IOService.appendToFile(ovitoFilePath, ovitoOutputData);
  }

  private boolean validParametersRange(final double length,
                                       final double width,
                                       final double diameterOpening) {
    return length > width && width > diameterOpening;
  }

  private double randomDiameter() {
    return RandomService.randomDouble(MIN_DIAMETER, MAX_DIAMETER);
  }
}
