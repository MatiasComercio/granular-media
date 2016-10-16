package ar.edu.itba.ss.granularmedia.core.helpers;

import ar.edu.itba.ss.granularmedia.models.Particle;
import ar.edu.itba.ss.granularmedia.models.StaticData;

import java.util.Collection;
import java.util.HashSet;

import static ar.edu.itba.ss.granularmedia.models.ParticleType.BORDER;

public class OutputSerializerHelper {
  private static final String NL = System.lineSeparator();
  private static final char SPLITTER = '\t';
  private static final int R = 0;
  private static final int G = 1;
  private static final int B = 2;

  private final Collection<Particle> ovitoBorderParticles;

  public OutputSerializerHelper(final StaticData staticData) {
    ovitoBorderParticles = generateOvitoBorderParticles(staticData.width(), staticData.length());
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
  public String ovitoOutput(final Collection<Particle> particles, long iteration) {
    final StringBuilder sb = new StringBuilder();
    final int N = particles.size() + ovitoBorderParticles.size();

    // (system + border) particles number
    sb.append(N).append(NL);
    // iterations' number
    sb.append(iteration).append(NL);
    // system's particles' data
    serializeParticles(particles, sb);
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

  private Collection<Particle> generateOvitoBorderParticles(final double width, final double length) {
    final Collection<Particle> particles = new HashSet<>();
    final Particle leftBottomParticle = Particle.builder(0, 0).id(-4).type(BORDER).build();
    final Particle rightBottomParticle = Particle.builder(width, 0).id(-3).type(BORDER).build();
    final Particle rightTopParticle = Particle.builder(width, length).id(-2).type(BORDER).build();
    final Particle leftTopParticle = Particle.builder(0, length).id(-1).type(BORDER).build();

    particles.add(leftBottomParticle);
    particles.add(rightBottomParticle);
    particles.add(rightTopParticle);
    particles.add(leftTopParticle);

    return particles;
  }

  private static StringBuilder serializeParticles(final Iterable<Particle> particles,
                                           final StringBuilder sb) {
    for (Particle particle : particles) {
      final double[] color = chooseColor(particle);

      // serialize particle
      serializeParticle(particle, color, sb);
    }
    return sb;
  }

  private static StringBuilder serializeParticle(final Particle particle,
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
            // type
            .append(particle.type()).append(SPLITTER);

    return sb.append(NL);
  }

  private static double[] chooseColor(final Particle particle) {
    final double[] color = new double[3];
    switch (particle.type()) {
      case COMMON:
        // red
        color[R] = 255;
        color[G] = color[B] = 0;
        break;
      case BORDER:
        // black
        color[R] = color[G] = color[B] = 0;
        break;
      default:
        // white
        color[R] = color[G] = color[B] = 255;
        break;
    }
    return color;
  }
}
