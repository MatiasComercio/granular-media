package ar.edu.itba.ss.granularmedia.models;

import org.immutables.builder.Builder;
import org.immutables.value.Value;

@Value.Immutable
@Value.Style(
        typeAbstract = "*Abs",
        typeImmutable = "*",
        get = ""
)
abstract class ParticleAbs {

  private static long idGen = 1;

  @Value.Default
  public long id() {
    return idGen ++;
  }

  @Value.Default
  @Value.Auxiliary
  public ParticleType type() { return ParticleType.COMMON; }

  @Value.Default
  @Value.Auxiliary
  public boolean isColliding() { return false; }

  @Builder.Parameter
  @Value.Auxiliary
  public abstract double x();

  @Builder.Parameter
  @Value.Auxiliary
  public abstract double y();

  @Value.Default
  @Value.Auxiliary
  public double mass() {
    return 0;
  }

  @Value.Default
  @Value.Auxiliary
  public double radio() {
    return 0;
  }

  @Value.Check
  void checkRadio() {
    if (radio() < 0) {
      throw new IllegalArgumentException("Radio should be >= 0");
    }
  }

  @Value.Default
  @Value.Auxiliary
  public double vx() {
    return 0;
  }

  @Value.Default
  @Value.Auxiliary
  public double vy() {
    return 0;
  }

  @Value.Default
  @Value.Auxiliary
  public double forceX() {
    return 0;
  }

  @Value.Default
  @Value.Auxiliary
  public double forceY() {
    return 0;
  }

  @Value.Derived
  @Value.Auxiliary
  public double speed() {
    return Math.sqrt(Math.pow(vx(), 2) + Math.pow(vy(), 2));
  }

  @Value.Check
  void checkSpeed() {
    if (speed() < 0) {
      throw new IllegalArgumentException("Speed (velocity's module) should be >= 0");
    }
  }

  @Value.Derived
  @Value.Auxiliary
  public double kineticEnergy() {
    return 1/2.0d * mass() * Math.pow(speed(), 2);
  }

  /**
   * Prints the immutable value {@code Particle} with attribute values.
   * @return A string representation of the value
   */
  @Override
  public String toString() {
    return "Particle{"
            + "id=" + id()
            + ", type= " + type()
            + ", x=" + x()
            + ", y=" + y()
            + ", radio=" + radio()
            + ", vx=" + vx()
            + ", vy=" + vy()
            + ", forceX=" + forceX()
            + ", forceY=" + forceY()
            + ", speed=" + speed()
            + ", mass=" + mass()
            + ", kinetic energy=" + kineticEnergy()
            + "}";
  }
}
