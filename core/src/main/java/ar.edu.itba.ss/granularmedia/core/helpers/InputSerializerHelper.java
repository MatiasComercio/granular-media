package ar.edu.itba.ss.granularmedia.core.helpers;

import ar.edu.itba.ss.granularmedia.models.Particle;
import ar.edu.itba.ss.granularmedia.models.ParticleType;
import ar.edu.itba.ss.granularmedia.models.StaticData;
import ar.edu.itba.ss.granularmedia.services.IOService;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Collection;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Scanner;
import java.util.stream.Stream;

import static ar.edu.itba.ss.granularmedia.services.IOService.ExitStatus.COULD_NOT_OPEN_INPUT_FILE;

public class InputSerializerHelper {

  public static StaticData loadStaticFile(final String staticFile) {
    final Path staticFilePath = Paths.get(staticFile);
    if (!IOService.openInputFile(staticFilePath)) {
      IOService.exit(COULD_NOT_OPEN_INPUT_FILE, staticFilePath);
      // should never reach here
      throw new IllegalStateException();
    }
    final StaticData staticData = InputSerializerHelper.readStaticFile(staticFilePath);
    IOService.closeInputFile(staticFilePath);
    return staticData;
  }

  public static Collection<Particle> loadDynamicData(final String dynamicFile) {
    final Path dynamicFilePath = Paths.get(dynamicFile);
    if (!IOService.openInputFile(dynamicFilePath)) {
      IOService.exit(COULD_NOT_OPEN_INPUT_FILE, dynamicFilePath);
      // should never reach here
      throw new IllegalStateException();
    }
    final Collection<Particle> particles = InputSerializerHelper.readDynamicFile(dynamicFilePath);
    IOService.closeInputFile(dynamicFilePath);
    return particles;
  }

  private static StaticData readStaticFile(final Path staticFilePath) {
    final Stream<String> stream = IOService.readLines(staticFilePath);
    final Iterator<String> streamLines = stream.iterator();
    final int n = IOService.parseAsInt(streamLines.next(), "<nParticles>");
    final double width = IOService.parseAsDouble(streamLines.next(), "<width>");
    final double length = IOService.parseAsDouble(streamLines.next(), "<length>");
    final double diameterOpening = IOService.parseAsDouble(streamLines.next(), "<diameterOpening>");
    final double mass = IOService.parseAsDouble(streamLines.next(), "<mass>");
    final double kn = IOService.parseAsDouble(streamLines.next(), "<kn>");
    final double kt = IOService.parseAsDouble(streamLines.next(), "<kt>");

    return StaticData.builder(n, width, length, diameterOpening, mass, kn, kt).build();
  }

  private static Collection<Particle> readDynamicFile(final Path dynamicFilePath) {
    final Stream<String> stream = IOService.readLines(dynamicFilePath);
    final Iterator<String> streamLines = stream.iterator();

    final int totalParticles = IOService.parseAsInt(streamLines.next(), "<nParticles>");

    final Collection<Particle> particles = new HashSet<>(totalParticles);
    for (int i = 0 ; i < totalParticles ; i++) {
      final Scanner numScanner = new Scanner(streamLines.next());
      final int id = numScanner.nextInt();
      final double x = numScanner.nextDouble(); // caught InputMismatchException
      final double y = numScanner.nextDouble(); // caught InputMismatchException
      final double vx = numScanner.nextDouble();
      final double vy = numScanner.nextDouble();
      final double forceX = numScanner.nextDouble();
      final double forceY = numScanner.nextDouble();
      numScanner.nextDouble(); // skip color[R]
      numScanner.nextDouble(); // skip color[G]
      numScanner.nextDouble(); // skip color[B]
      final double radio = numScanner.nextDouble();
      final double mass = numScanner.nextDouble();
      numScanner.nextDouble(); // skip value int
      final ParticleType type =  ParticleType.valueOf(numScanner.next());

      particles.add(Particle.builder(x,y).id(id).vx(vx).vy(vy).forceX(forceX).forceY(forceY)
              .radio(radio).mass(mass).type(type).build());
    }
    return particles;
  }
}
