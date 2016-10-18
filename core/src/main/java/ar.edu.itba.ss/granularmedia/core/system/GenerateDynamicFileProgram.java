package ar.edu.itba.ss.granularmedia.core.system;

import ar.edu.itba.ss.granularmedia.core.helpers.InputSerializerHelper;
import ar.edu.itba.ss.granularmedia.core.helpers.OutputSerializerHelper;
import ar.edu.itba.ss.granularmedia.interfaces.MainProgram;
import ar.edu.itba.ss.granularmedia.models.Particle;
import ar.edu.itba.ss.granularmedia.models.StaticData;
import ar.edu.itba.ss.granularmedia.services.IOService;
import ar.edu.itba.ss.granularmedia.services.RandomService;
import ar.edu.itba.ss.granularmedia.services.factories.ParticleFactory;

import java.nio.file.Path;
import java.util.Collection;

import static ar.edu.itba.ss.granularmedia.services.IOService.ExitStatus.*;

public class GenerateDynamicFileProgram implements MainProgram {
  // condition constants
  private static final boolean OVERLAP_ALLOWED = false;
  private static final int MAX_OVERLAP_TRIES = 100;

  // non-magic number constants
  private static final double ZERO = 0;

  private static final String DEFAULT_OUTPUT_FOLDER = "output";
  private static final String DEFAULT_DYNAMIC_FILE_NAME = "dynamic";
  private static final String DEFAULT_DAT_FILE_EXTENSION = ".dat";

  // gen static args indexes
  private static final int I_STATIC_FILE = 2;

  private static final int N_ARGS_EXPECTED = 3;

  @Override
  public void run(final String[] args) {
    System.out.println("Generating dynamic file...");

    if (args.length < N_ARGS_EXPECTED) {
      IOService.exit(BAD_N_ARGUMENTS, null);
      // should never reach here
      throw new IllegalStateException();
    }

    final StaticData staticData = InputSerializerHelper.loadStaticFile(args[I_STATIC_FILE]);
    final Collection<Particle> particles = initializeSystemParticles(staticData);

    final String serializedStaticData = OutputSerializerHelper.dynamicOutput(particles);
    final Path pathToStaticFile =
            IOService.createOutputFile(DEFAULT_OUTPUT_FOLDER, DEFAULT_DYNAMIC_FILE_NAME, DEFAULT_DAT_FILE_EXTENSION);
    IOService.openOutputFile(pathToStaticFile, true);
    IOService.appendToFile(pathToStaticFile, serializedStaticData);
    IOService.closeOutputFile(pathToStaticFile);
    System.out.println("[DONE]");
  }

  private Collection<Particle> initializeSystemParticles(final StaticData staticData) {
    final ParticleFactory particleFactory = ParticleFactory.getInstance();

    if (!validParametersRange(staticData.length(), staticData.width(), staticData.diameterOpening())) {
      IOService.exit(VALIDATION_FAILED, "length > width > diameterOpening");
    }

    final double[] radios = new double[staticData.N()];
    double diameter;
    final double minDiameter = staticData.minDiameter();
    final double maxDiameter = staticData.maxDiameter();
    for (int i = 0  ; i < staticData.N() ; i++) {
      diameter = randomDiameter(minDiameter, maxDiameter);
      radios[i] = diameter/2;
    }

    final Particle leftBottomParticle = particleFactory.create(ZERO, staticData.fallLength());
    final Particle rightTopParticle =
            particleFactory.create(staticData.width() + ZERO, staticData.fallLength() + staticData.length() + ZERO);

    return particleFactory.randomPoints(
            leftBottomParticle,
            rightTopParticle,
            radios, staticData.mass(),
            OVERLAP_ALLOWED,
            MAX_OVERLAP_TRIES);
  }

  private boolean validParametersRange(final double length,
                                       final double width,
                                       final double diameterOpening) {
    return length > width && width > diameterOpening;
  }

  private double randomDiameter(final double minDiameter, final double maxDiameter) {
    return RandomService.randomDouble(minDiameter, maxDiameter);
  }
}
