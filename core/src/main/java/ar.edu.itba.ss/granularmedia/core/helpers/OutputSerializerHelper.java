package ar.edu.itba.ss.granularmedia.core.helpers;

import ar.edu.itba.ss.granularmedia.models.Particle;
import ar.edu.itba.ss.granularmedia.models.StaticData;
import ar.edu.itba.ss.granularmedia.models.Wall;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.Collection;
import java.util.HashSet;

import static ar.edu.itba.ss.granularmedia.models.ParticleType.BORDER;
import static ar.edu.itba.ss.granularmedia.models.ParticleType.SPAWN;

public class OutputSerializerHelper {
  private static final Logger LOGGER = LoggerFactory.getLogger(OutputSerializerHelper.class);
  private static final double ZERO = 0;
  private static final String NL = System.lineSeparator();
  private static final char SPLITTER = '\t';
  private static final int R = 0;
  private static final int G = 1;
  private static final int B = 2;

  private static int OVITO_ID_GEN = 0;

  private final Collection<Particle> ovitoBorderParticles;

  public OutputSerializerHelper(final StaticData staticData) {
    ovitoBorderParticles = generateOvitoBorderParticles(staticData);
  }

  /**
   * Prepares output for Ovito file serializing all the needed information
   * (system particles, border particles, static data, number of iteration, etc.)
   * according to the given params, for the given iteration
   *
   * @param particles system particles
   * @param iteration number of current iteration
   * @return a serialized set of data to save at the ovito's file for the given iteration
   */
  public String ovitoOutput(final Collection<Particle> particles,
                            final Collection<Wall> walls,
                            long iteration) {
    final StringBuilder sb = new StringBuilder();
    // each wall represented with 2 particles
    final int N = particles.size() + walls.size() * 2 + ovitoBorderParticles.size();

    // (system + border) particles number
    sb.append(N).append(NL);
    // iterations' number
    sb.append(iteration).append(NL);
    // system's particles' data
    serializeParticles(particles, sb);
    // system's walls' data
    serializeWalls(walls, sb);
    // ovito's border particles
    serializeParticles(ovitoBorderParticles, sb);
    return sb.toString();
  }

  public static String staticOutput(final StaticData staticData) {
    @SuppressWarnings("StringBufferReplaceableByString")
    final StringBuilder sb = new StringBuilder();
    sb      .append(staticData.N()).append(NL)
            .append(staticData.width()).append(NL)
            .append(staticData.length()).append(NL)
            .append(staticData.diameterOpening()).append(NL)
            .append(staticData.mass()).append(NL)
            .append(staticData.kn()).append(NL)
            .append(staticData.kt()).append(NL);
    return sb.toString();
  }

  public static String dynamicOutput(final Collection<Particle> particles) {
    final StringBuilder sb = new StringBuilder();
    // system particles number
    sb.append(particles.size()).append(NL);
    // system's particles' data
    serializeParticles(particles, sb);
    return sb.toString();
  }

  private Collection<Particle> generateOvitoBorderParticles(final StaticData staticData) {
    // check StaticDataAbs class documentation to see map's display
    final Collection<Particle> particles = new HashSet<>();

    // enclosing system
    final Particle leftBottomParticle =
            Particle.builder(ZERO, ZERO).id(nextOvitoId()).type(BORDER).build();
    final Particle rightBottomParticle =
            Particle.builder(staticData.width(), ZERO).id(nextOvitoId()).type(BORDER).build();
    final Particle rightTopParticle =
            Particle.builder(staticData.width(), staticData.totalSystemLength()).id(nextOvitoId()).type(BORDER).build();
    final Particle leftTopParticle =
            Particle.builder(ZERO, staticData.totalSystemLength()).id(nextOvitoId()).type(BORDER).build();

    // for drawing spawn area
    final double spawnLength = staticData.fallLength() + staticData.length();
    final Particle leftSpawnParticle =
            Particle.builder(ZERO, spawnLength).id(nextOvitoId()).type(SPAWN).build();
    final Particle rightSpawnParticle =
            Particle.builder(staticData.width(), spawnLength).id(nextOvitoId()).type(SPAWN).build();


    particles.add(leftBottomParticle);
    particles.add(rightBottomParticle);
    particles.add(rightTopParticle);
    particles.add(leftTopParticle);

    particles.add(leftSpawnParticle);
    particles.add(rightSpawnParticle);

    return particles;
  }

  private long nextOvitoId() {
    return --OVITO_ID_GEN;
  }

  private static StringBuilder serializeParticles(final Iterable<Particle> particles,
                                         final StringBuilder sb) {
    for (Particle particle : particles) {
      final double[] color = chooseColor(particle);

      // serializeWalls particle
      serialize(particle, color, sb);
    }
    return sb;
  }

  private static StringBuilder serialize(final Particle particle,
                                         final double[] color,
                                         final StringBuilder sb) {
    sb.append(
            // id
            particle.id()).append(SPLITTER)
            // position
            .append(particle.x()).append(SPLITTER).append(particle.y()).append(SPLITTER)
            // velocity
            .append(particle.vx()).append(SPLITTER).append(particle.vy()).append(SPLITTER)
            // force
            .append(particle.forceX()).append(SPLITTER).append(particle.forceY()).append(SPLITTER)
            // R G B color
            .append(color[R]).append(SPLITTER)
            .append(color[G]).append(SPLITTER)
            .append(color[B]).append(SPLITTER)
            // radio
            .append(particle.radio()).append(SPLITTER)
            // mass
            .append(particle.mass()).append(SPLITTER)
            // class
            .append(particle.type().getCode()).append(SPLITTER)
            // type
            .append(particle.type()).append(SPLITTER);

    return sb.append(NL);
  }

  private static StringBuilder serializeWalls(final Iterable<Wall> walls,
                                              final StringBuilder sb) {
    for (final Wall wall : walls) {
      final Particle start = wall.start();
      final double[] startColor = chooseColor(start);
      // serializeWalls particle
      serialize(start, startColor, sb);

      final Particle end = wall.end();
      final double[] endColor = chooseColor(end);
      // serializeWalls particle
      serialize(end, endColor, sb);
    }
    return sb;
  }

  private static double[] chooseColor(final Particle particle) {
    final double[] color = new double[3];
    switch (particle.type()) {
      case COMMON:
        // red increasing with pressure
        // blue decreasing with pressure
        final double pressure = particle.pressure() / Particle.getMaxPressure();
        color[R] = pressure ;
        color[B] = 1 - color[R];
        color[G] = 0;
        break;
      case BORDER: case OPENING_LEFT: case OPENING_RIGHT:
        // black
        color[R] = color[G] = color[B] = 0;
        break;
      case SPAWN:
        // grey
        color[R] = color[G] = color[B] = 0.5;
        break;
      default:
        // white
        color[R] = color[G] = color[B] = 255;
        break;
    }
    return color;
  }
}
