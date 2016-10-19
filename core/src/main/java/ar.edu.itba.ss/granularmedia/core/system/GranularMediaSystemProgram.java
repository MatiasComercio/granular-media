package ar.edu.itba.ss.granularmedia.core.system;

import ar.edu.itba.ss.granularmedia.core.helpers.InputSerializerHelper;
import ar.edu.itba.ss.granularmedia.core.helpers.OutputSerializerHelper;
import ar.edu.itba.ss.granularmedia.core.system.integration.Gear5GranularMediaSystemData;
import ar.edu.itba.ss.granularmedia.core.system.integration.GearGranularMediaSystem;
import ar.edu.itba.ss.granularmedia.interfaces.MainProgram;
import ar.edu.itba.ss.granularmedia.interfaces.TimeDrivenSimulationSystem;
import ar.edu.itba.ss.granularmedia.models.Particle;
import ar.edu.itba.ss.granularmedia.models.StaticData;
import ar.edu.itba.ss.granularmedia.models.Wall;
import ar.edu.itba.ss.granularmedia.models.WallType;
import ar.edu.itba.ss.granularmedia.services.IOService;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.nio.file.Path;
import java.time.LocalDateTime;
import java.util.*;

import static ar.edu.itba.ss.granularmedia.services.IOService.ExitStatus.BAD_N_ARGUMENTS;

public class GranularMediaSystemProgram implements MainProgram {
  private static final Logger LOGGER = LoggerFactory.getLogger(GranularMediaSystemProgram.class);

  // non-magic number constants
  private static final double ZERO = 0;

  // file constants
  private static final String DEFAULT_OUTPUT_FOLDER = "output";
  private static final String OVITO_FILE_EXTENSION = ".xyz";
  private static final String DEFAULT_OVITO_FILE_NAME = "ovito";
  private static final String STATISTICS_FILE_EXTENSION = ".csv";
  private static final String DEFAULT_KINETIC_ENERGY_FILE_NAME = "kinetic_energy";
  private static final String DEFAULT_SYSTEM_STOPPED_FILE_NAME = "system_stopped";
  private static final String DEFAULT_FLOW_FILE_NAME = "flow";
  private static final String DEFAULT_MEDIA_FLOW_FILE_NAME = "flow_media";

  private static final double MS_TO_S = 1/1000.0;
  private static final double DELTA_LOG = 0.025;
  private static final double ERROR_TOLERANCE = 1e-8;

  // run args index
  private static final int I_STATIC_DATA = 1;
  private static final int I_DYNAMIC_DATA = 2;
  private static final int I_SIMULATION_TIME = 3;
  private static final int I_DELTA_1 = 4;
  private static final int I_DELTA_2 = 5;
  private static final int N_ARGS_EXPECTED = 6;

  private final Path pathToOvitoFile;
  private final Path pathToKineticEnergyFile;
  private final Path pathToFlowFile;

  public GranularMediaSystemProgram() {
    this.pathToOvitoFile =
            IOService.createOutputFile(DEFAULT_OUTPUT_FOLDER, DEFAULT_OVITO_FILE_NAME, OVITO_FILE_EXTENSION);
    this.pathToKineticEnergyFile =
            IOService.createOutputFile(DEFAULT_OUTPUT_FOLDER,
                    DEFAULT_KINETIC_ENERGY_FILE_NAME, STATISTICS_FILE_EXTENSION);
    this.pathToFlowFile =
            IOService.createOutputFile(DEFAULT_OUTPUT_FOLDER,
                    DEFAULT_FLOW_FILE_NAME, STATISTICS_FILE_EXTENSION);
  }

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

    // add opening extremes that are inside the system as particles with
    // infinite mass so as to improve collisions
    systemParticles.addAll(getOpeningWallsParticles(systemWalls));

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

    // close resources
    IOService.closeOutputFile(pathToOvitoFile);
    IOService.closeOutputFile(pathToKineticEnergyFile);
    IOService.closeOutputFile(pathToFlowFile);
  }

  // private
  private void startSimulation(final TimeDrivenSimulationSystem<Gear5GranularMediaSystemData> granularMediaSystem,
                               final double dt, final double simulationTime, final double delta2,
                               final OutputSerializerHelper outputSerializerHelper) {
    final double startTime = System.currentTimeMillis();

    long step = 0;
    long logStep = 0;
    double currentTime = 0;
    boolean considerKineticEnergy = false;
    double kineticEnergy; // initialization not needed
    while (currentTime < simulationTime) {
      // choose output action based on given parameters
      if (currentTime >= (delta2 * step)) {
        outputSystem(granularMediaSystem.getSystemData(), step, currentTime, outputSerializerHelper);
        step ++;
      }

      if (currentTime >= (DELTA_LOG * logStep)) {
        System.out.printf(
                "\tClock: %s; Current simulation time: %f ; Final simulation time: %f ; Current Kinetic Energy: %e\n",
                LocalDateTime.now(), currentTime, simulationTime, granularMediaSystem.getSystemData().kineticEnergy());
        logStep ++;
      }

      // evolve system
      granularMediaSystem.evolveSystem(dt);

      // advance time and count the current step
      currentTime += dt;

      appendToFlow(pathToFlowFile, granularMediaSystem.getSystemData().nParticlesJustFlowed(), step, currentTime, outputSerializerHelper);

      // if no more particles are moving => system's evolution is finished
      kineticEnergy = granularMediaSystem.getSystemData().kineticEnergy();
      if (!considerKineticEnergy) { // start considering kinetic energy after it overcomes the default ERROR_TOLERANCE
        considerKineticEnergy = kineticEnergy > ERROR_TOLERANCE;
      }
      if (considerKineticEnergy && kineticEnergy < ERROR_TOLERANCE) {
        systemStopped(step, currentTime);
        break;
      }
    }

    if (!Double.valueOf(simulationTime).equals(ZERO)) {
      outputMediaFlow(granularMediaSystem.getSystemData().nParticlesFlowed() / simulationTime);
    }

    final double endTime = System.currentTimeMillis();
    final double simulationDuration = endTime - startTime;
    LOGGER.info("Total simulation time: {} s", simulationDuration * MS_TO_S);
    System.out.printf("Total simulation time: %f s\n", simulationDuration * MS_TO_S);
  }

  private void outputMediaFlow(final double mediaFlow) {
    final Path pathToOutputMediaFlowFile =
            IOService.createOutputFile(DEFAULT_OUTPUT_FOLDER, DEFAULT_MEDIA_FLOW_FILE_NAME, STATISTICS_FILE_EXTENSION);
    IOService.appendToFile(pathToOutputMediaFlowFile, String.valueOf(mediaFlow));
    IOService.closeOutputFile(pathToOutputMediaFlowFile);
    System.out.println("Media Flow: " + mediaFlow);
  }

  private void systemStopped(final long step, final double currentTime) {
    System.out.printf("\tSystem has reached the stop condition at time: %fs.%s",
            currentTime, System.lineSeparator());
    final Path pathToSystemStoppedFile =
            IOService.createOutputFile(DEFAULT_OUTPUT_FOLDER,
                    DEFAULT_SYSTEM_STOPPED_FILE_NAME, STATISTICS_FILE_EXTENSION);
    final String fileMsg = step + ", " + currentTime;
    IOService.appendToFile(pathToSystemStoppedFile, fileMsg);
    IOService.closeOutputFile(pathToSystemStoppedFile);
  }

  private Collection<Particle> getOpeningWallsParticles(final Collection<Wall> walls) {
    final Collection<Particle> openingParticles = new HashSet<>();

    for (final Wall wall : walls) {
      if (wall.type() == WallType.HORIZONTAL_LEFT) {
        openingParticles.add(wall.end());
      } else if (wall.type() == WallType.HORIZONTAL_RIGHT) {
        openingParticles.add(wall.start());
      }
    }

    return openingParticles;
  }

  private StaticData loadStaticData(final String[] args) {
    final StaticData staticData = InputSerializerHelper.loadStaticFile(args[I_STATIC_DATA]);
    final double simulationTime = IOService.parseAsDouble(args[I_SIMULATION_TIME], "<simulation_time>");
    final double delta1 = IOService.parseAsDouble(args[I_DELTA_1], "<delta_1>");
    final double delta2 = IOService.parseAsDouble(args[I_DELTA_2], "<delta_2>");

    return staticData.withSimulationTime(simulationTime).withDelta1(delta1).withDelta2(delta2);
  }

  private void outputSystem(final Gear5GranularMediaSystemData systemData,
                            final long step, final double currentTime,
                            final OutputSerializerHelper outputSerializerHelper) {
    // print system after printStepGap dt units
    appendToOvito(pathToOvitoFile, systemData.particles(), systemData.walls(), step, outputSerializerHelper);
    appendToKineticEnergy(pathToKineticEnergyFile, systemData.kineticEnergy(), step, currentTime);
  }

  private void appendToFlow(final Path pathToFlowFile, final long nParticlesFlowed,
                            final long step, final double currentTime,
                            final OutputSerializerHelper outputSerializerHelper) {
    // if there weren't particles that flowed, outputSerializerHelper takes this into account
    final String flowData = outputSerializerHelper.flowSerializer(step, currentTime, nParticlesFlowed);
    IOService.appendToFile(pathToFlowFile, flowData);
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

  private void appendToOvito(final Path ovitoFilePath,
                             final Collection<Particle> particleSet,
                             final Collection<Wall> walls,
                             final long iteration,
                             final OutputSerializerHelper outputSerializerHelper) {
    final String ovitoOutputData = outputSerializerHelper.ovitoOutput(particleSet, walls, iteration);
    IOService.appendToFile(ovitoFilePath, ovitoOutputData);
  }

  private void appendToKineticEnergy(final Path pathToKineticEnergyFile,
                                     final double kineticEnergy,
                                     final long step,
                                     final double currentTime) {
    final String kineticOutputData = step + ", " + currentTime + ", " + kineticEnergy + System.lineSeparator();
    IOService.appendToFile(pathToKineticEnergyFile, kineticOutputData);
  }
}
