package ar.edu.itba.ss.granularmedia.models;

import org.immutables.builder.Builder;
import org.immutables.value.Value;

import static ar.edu.itba.ss.granularmedia.models.ParticleType.*;

@Value.Immutable
@Value.Style(
        typeAbstract = "*Abs",
        typeImmutable = "*",
        get = ""
)
public abstract class WallAbs {
  private static final Double ZERO = 0d;

  // c1
  @Value.Default
  @Builder.Parameter
  public double xFrom() {
    return 0;
  }
  @Value.Default
  @Builder.Parameter
  public double yFrom() {
    return 0;
  }

  @Value.Derived
  @Value.Auxiliary
  public Vector2D c1() {
    return Vector2D.builder(xFrom(), yFrom()).build();
  }

  // c2
  @Value.Default
  @Builder.Parameter
  public double xTo() {
    return 0;
  }
  @Value.Default
  @Builder.Parameter
  public double yTo() {
    return 0;
  }

  @Value.Derived
  @Value.Auxiliary
  public Vector2D c2() {
    return Vector2D.builder(xTo(), yTo()).build();
  }

  @Value.Derived
  @Value.Auxiliary
  public Vector2D asVector() {
    return Vector2D.builder(xTo() - xFrom(), yTo() - yFrom()).build();
  }

  @Value.Derived
  @Value.Auxiliary
  public Vector2D tangentialVersor() {
    return asVector().toVersor();
  }

  /**
   *
   * @return right normal versor, considered from the tangential versor, i.e., normal versor as with the right hand rule
   */
  @Value.Derived
  @Value.Auxiliary
  public Vector2D rightNormalVersor() {
    final Vector2D tangentialVersor = tangentialVersor();
    return Vector2D.builder(tangentialVersor.y(), - tangentialVersor.x()).build();
  }

  /**
   *
   * If this vector were the corresponding one for a given vector => rotate (multiply by -1) the tangential versor
   * of this wall so as to match the right hand rule
   * @return right left versor, considered from the tangential versor
   */
  @Value.Derived
  @Value.Auxiliary
  public Vector2D leftNormalVersor() {
    final Vector2D tangentialVersor = tangentialVersor();
    return Vector2D.builder(- tangentialVersor.y(), tangentialVersor.x()).build();
  }


  @Value.Derived
  public double length() {
    return Math.abs(yTo() - yFrom());
  }

  @Value.Derived
  public double width() {
    return Math.abs(xTo() - xFrom());
  }

  @Value.Default
  @Value.Auxiliary
  public WallType type() {
    final double length = length();
    final double width = width();
    if (!ZERO.equals(length * width)) {
      return WallType.DIAGONAL;
    }
    if (length > 0) {
      return WallType.VERTICAL;
    }
    // only Horizontal Wall remains
    if (ZERO.equals(xFrom())) {
      return WallType.HORIZONTAL_LEFT; // only matches start position
    } else {
      return WallType.HORIZONTAL_RIGHT;
    }
  }

  @Value.Check
  void checkParameters() {
    if (ZERO.equals(length() + width())) {
      throw new IllegalArgumentException("You should not create a Wall without " +
              "width neither length");
    }
  }

  @Value.Derived
  public Particle start() {
    final Particle.Builder builder = Particle.builder(xFrom(), yFrom());
    chooseType(builder);
    return builder.build();
  }

  @Value.Derived
  public Particle end() {
    final Particle.Builder builder = Particle.builder(xTo(), yTo());
    chooseType(builder);
    return builder.build();
  }

  private void chooseType(final Particle.Builder builder) {
    switch (type()) {
      case HORIZONTAL_LEFT:
        builder.type(OPENING_LEFT);
        break;
      case HORIZONTAL_RIGHT:
        builder.type(OPENING_RIGHT);
        break;
      default:
        builder.type(WALL);
        break;
    }
  }
}
