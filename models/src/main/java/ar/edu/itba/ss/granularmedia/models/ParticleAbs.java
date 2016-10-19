package ar.edu.itba.ss.granularmedia.models;

import org.immutables.builder.Builder;
import org.immutables.value.Value;

@Value.Immutable
@Value.Style(
        typeAbstract = "*Abs",
        typeImmutable = "*",
        get = ""
)
public abstract class ParticleAbs {
  private static double maxPressure = 0;

  private static long idGen = 1;

  private double normalForce = 0;

  private boolean hasFlowedOut = false;

  public boolean hasFlowedOut() {
    return hasFlowedOut;
  }

  public void hasFlowedOut(final boolean hasFlowedOut) {
    this.hasFlowedOut = hasFlowedOut;
  }

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

  private double normalForce() {
    return normalForce;
  }

  public void normalForce(final double normalForce) {
    this.normalForce = normalForce;
  }

  public void increaseNormalForce(final double normalForce) {
    this.normalForce += normalForce;
  }

  @Value.Derived
  @Value.Auxiliary
  double perimeter() {
    return 2 * Math.PI * radio();
  }

  public double pressure() {
    return normalForce() / perimeter();
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

  @Value.Derived
  @Value.Auxiliary
  public Vector2D r0() {
    return Vector2D.builder(x(), y()).build();
  }

  @Value.Derived
  @Value.Auxiliary
  public Vector2D r1() {
    return Vector2D.builder(vx(), vy()).build();
  }

  @Value.Derived
  @Value.Auxiliary
  public Vector2D r2() {
    if (Double.valueOf(0).equals(mass())) {
      return Vector2D.builder(0, 0).build();
    }
    return Vector2D.builder(forceX()/mass(), forceY()/mass()).build();
  }

  public static double getMaxPressure() {
    return maxPressure;
  }

  public static void setMaxPressure(final double maxPressure) {
    ParticleAbs.maxPressure = maxPressure;
  }

  public Particle respawn(final double x, final double y, final double forceX, final double forceY) {
    return Particle.builder(x, y).id(id())
            .radio(radio()).mass(mass()).forceX(forceX).forceY(forceY).type(type())
            .build();
  }

  public Particle update(final Vector2DAbs uP, final Vector2DAbs uV, final Vector2DAbs uF) {
    if (pressure() > maxPressure) {
      maxPressure = pressure();
    }

    final Particle particle = Particle.builder(uP.x(), uP.y())
            .vx(uV.x()).vy(uV.y())
            .forceX(uF.x()).forceY(uF.y())
            .id(id())
            .type(type())
            .isColliding(isColliding())
            .mass(mass())
            .radio(radio())
            .build();
    particle.normalForce(normalForce()); // update normal force
    particle.hasFlowedOut(hasFlowedOut()); // update has flowed out
    return particle;
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
