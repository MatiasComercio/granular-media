package ar.edu.itba.ss.granularmedia.core;

import ar.edu.itba.ss.granularmedia.core.helpers.OutputSerializerHelper;
import ar.edu.itba.ss.granularmedia.models.Particle;
import ar.edu.itba.ss.granularmedia.models.StaticData;
import ar.edu.itba.ss.granularmedia.services.IOService;
import ar.edu.itba.ss.granularmedia.services.ParticleFactory;
import ar.edu.itba.ss.granularmedia.services.RandomService;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.nio.file.Path;
import java.util.Set;

import static ar.edu.itba.ss.granularmedia.services.IOService.ExitStatus.*;

public class Main {
  private static final Logger LOGGER = LoggerFactory.getLogger(Main.class);
  private static final int N_PARTICLES = 100;
  private static final double LENGTH = 15;
  private static final double WIDTH = 10;
  private static final double DIAMETER_OPENING = 5;
  private static final double ZERO = 0;

  private static final double MIN_DIAMETER = DIAMETER_OPENING /7;
  private static final double MAX_DIAMETER = DIAMETER_OPENING /5;

  private static final boolean OVERLAP_ALLOWED = false;
  private static final int MAX_OVERLAP_TRIES = 100;

  private static final long FIRST_ITERATION = 0;

  // Files
  private static final String OVITO_FILE_EXTENSION = ".xyz";
  private static final String DEFAULT_OUTPUT_FOLDER = "output";
  private static final String DEFAULT_OVITO_FILE_NAME = "ovito";

  public static void main(String[] args) {
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

    final Particle leftBottomParticle = particleFactory.create(ZERO, ZERO);
    final Particle rightTopParticle = particleFactory.create(WIDTH + ZERO, LENGTH + ZERO);

    LOGGER.debug("Creating particles...");
    final Set<Particle> particleSet = particleFactory.randomPoints(
            leftBottomParticle, rightTopParticle, radios, OVERLAP_ALLOWED, MAX_OVERLAP_TRIES);
    LOGGER.debug("[DONE] - Creating particles");

    LOGGER.debug("Writing ovito...");

    final OutputSerializerHelper outputSerializerHelper =
            new OutputSerializerHelper(StaticData.builder(N_PARTICLES, WIDTH, LENGTH).build());

    writeToOvito(DEFAULT_OUTPUT_FOLDER, DEFAULT_OVITO_FILE_NAME, particleSet, FIRST_ITERATION, outputSerializerHelper);
    LOGGER.debug("[DONE] - Writing ovito");
  }

  private static void writeToOvito(final String defaultOutputFolder,
                                   final String defaultOvitoFileName,
                                   final Set<Particle> particleSet,
                                   final long iteration,
                                   final OutputSerializerHelper outputSerializerHelper) {
    final String ovitoFile = defaultOvitoFileName + OVITO_FILE_EXTENSION;
    final Path ovitoFilePath = IOService.createFile(defaultOutputFolder, ovitoFile);

    final String ovitoOutputData = outputSerializerHelper.ovitoOutput(particleSet, iteration);
    IOService.appendToFile(ovitoFilePath, ovitoOutputData);
  }

  private static boolean validParametersRange(final double length,
                                              final double width,
                                              final double diameterOpening) {
    return length > width && width > diameterOpening;
  }

  private static double randomDiameter() {
    return RandomService.randomDouble(MIN_DIAMETER, MAX_DIAMETER);
  }


}
