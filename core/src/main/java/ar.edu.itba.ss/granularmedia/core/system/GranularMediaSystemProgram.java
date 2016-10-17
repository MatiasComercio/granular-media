package ar.edu.itba.ss.granularmedia.core.system;

import ar.edu.itba.ss.granularmedia.core.helpers.InputSerializerHelper;
import ar.edu.itba.ss.granularmedia.core.helpers.OutputSerializerHelper;
import ar.edu.itba.ss.granularmedia.core.system.integration.GearGranularMediaSystem;
import ar.edu.itba.ss.granularmedia.core.system.integration.GearGranularMediaSystem.Gear5GranularMediaSystemData;
import ar.edu.itba.ss.granularmedia.interfaces.MainProgram;
import ar.edu.itba.ss.granularmedia.interfaces.TimeDrivenSimulationSystem;
import ar.edu.itba.ss.granularmedia.models.Particle;
import ar.edu.itba.ss.granularmedia.models.StaticData;
import ar.edu.itba.ss.granularmedia.models.Wall;
import ar.edu.itba.ss.granularmedia.services.IOService;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.nio.file.Path;
import java.time.LocalDateTime;
import java.util.Collection;
import java.util.HashSet;

import static ar.edu.itba.ss.granularmedia.services.IOService.ExitStatus.BAD_N_ARGUMENTS;
import static ar.edu.itba.ss.granularmedia.services.IOService.ExitStatus.COULD_NOT_OPEN_OUTPUT_FILE;

public class GranularMediaSystemProgram implements MainProgram {
  private static final Logger LOGGER = LoggerFactory.getLogger(GranularMediaSystemProgram.class);

  // non-magic number constants
  private static final double ZERO = 0;

  // file constants
  private static final String OVITO_FILE_EXTENSION = ".xyz";
  private static final String DEFAULT_OUTPUT_FOLDER = "output";
  private static final String DEFAULT_OVITO_FILE_NAME = "ovito";
  private static final double MS_TO_S = 1/1000.0;
  private static final double DELTA_LOG = 0.5;

  // run args index
  private static final int I_STATIC_DATA = 1;
  private static final int I_DYNAMIC_DATA = 2;
  private static final int I_SIMULATION_TIME = 3;
  private static final int I_DELTA_1 = 4;
  private static final int I_DELTA_2 = 5;
  private static final int N_ARGS_EXPECTED = 6;

  @Override
  public void run(final String[] args) {
    if (args.length < N_ARGS_EXPECTED) {
      IOService.exit(BAD_N_ARGUMENTS, null);
      // should never reach here
      throw new IllegalStateException();
    }

    final StaticData staticData = loadStaticData(args);

    // system's particles
    final Collection<Particle> systemParticles =
            InputSerializerHelper.loadDynamicData(args[I_DYNAMIC_DATA]);

    // system's walls
    final Collection<Wall> systemWalls = initializeSystemWalls(staticData);

    final TimeDrivenSimulationSystem<Gear5GranularMediaSystemData> granularMediaSystem =
            new GearGranularMediaSystem(systemParticles, systemWalls, staticData);

    // helper to write ovito file
    final OutputSerializerHelper outputSerializerHelper = new OutputSerializerHelper(staticData);

    // default delta time
    final double defaultDelta1 = .1 * Math.sqrt(staticData.mass()/staticData.kn());
    final double dt = Math.min(defaultDelta1, staticData.delta1());
    System.out.printf("Chosen dt: %es\n", dt);
    System.out.println(staticData);
    System.out.println("Real system particles: " + systemParticles.size());

    // simulation itself
    System.out.println("Running simulation...");
    startSimulation(granularMediaSystem, dt, staticData.simulationTime(), staticData.delta2(), outputSerializerHelper);
    System.out.println("[DONE]");
  }

  // private
  private StaticData loadStaticData(final String[] args) {
    final StaticData staticData = InputSerializerHelper.loadStaticFile(args[I_STATIC_DATA]);
    final double simulationTime = IOService.parseAsDouble(args[I_SIMULATION_TIME], "<simulation_time>");
    final double delta1 = IOService.parseAsDouble(args[I_DELTA_1], "<delta_1>");
    final double delta2 = IOService.parseAsDouble(args[I_DELTA_2], "<delta_2>");

    return staticData.withSimulationTime(simulationTime).withDelta1(delta1).withDelta2(delta2);
  }

  private void startSimulation(final TimeDrivenSimulationSystem<Gear5GranularMediaSystemData> granularMediaSystem,
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
                granularMediaSystem.getSystemData().walls(),
                step++, outputSerializerHelper);
        if (currentTime >= (DELTA_LOG * logStep)) {
          System.out.printf("\tClock: %s; Current simulation time: %f ; Final simulation time: %f\n",
                  LocalDateTime.now(), currentTime, simulationTime);
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
    System.out.printf("Total simulation time: %f s\n", simulationDuration * MS_TO_S);
  }

  private Collection<Wall> initializeSystemWalls(final StaticData staticData) {
    final Collection<Wall> systemWalls = new HashSet<>();

    final Wall leftVerticalWall = Wall.builder(ZERO, ZERO, ZERO, staticData.totalSystemLength()).build();
    final Wall rightVerticalWall = Wall.builder(staticData.width(), ZERO, staticData.width(), staticData.totalSystemLength()).build();

    final double horizontalWallWidth = (staticData.width()-staticData.diameterOpening()) / 2;

    @SuppressWarnings("UnnecessaryLocalVariable")
    final double xFromLeftHorizontalWall = ZERO;
    final double xToLeftHorizontalWall = xFromLeftHorizontalWall + horizontalWallWidth;

    final double xFromRightHorizontalWall = (staticData.width() + staticData.diameterOpening()) / 2;
    final double xToRightHorizontalWall = xFromRightHorizontalWall + horizontalWallWidth;

    final Wall leftBottomHorizontalWall =
            Wall.builder(xFromLeftHorizontalWall, staticData.fallLength(),
                    xToLeftHorizontalWall, staticData.fallLength()).build();
    final Wall rightBottomHorizontalWall =
            Wall.builder(xFromRightHorizontalWall, staticData.fallLength(),
                    xToRightHorizontalWall, staticData.fallLength()).build();

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
                             final Collection<Wall> walls,
                             final long iteration,
                             final OutputSerializerHelper outputSerializerHelper) {
    final String ovitoOutputData = outputSerializerHelper.ovitoOutput(particleSet, walls, iteration);
    IOService.appendToFile(ovitoFilePath, ovitoOutputData);
  }
}
