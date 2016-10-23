package ar.edu.itba.ss.granularmedia.models;

import org.immutables.builder.Builder;
import org.immutables.value.Value;

/**
 * <pre>
                         |                      |                              |
                         |      RESPAWN_LENGTH  |                              |
                         |                     _|                              |_
                         |                      |                              |
                         |                      |                              |
    totalSystemLength    |      length          |                              |
                         |                      |                              |
                         |                     _|___________        ___________|_
                         |                      |        diameterOpening       |
                         |      FALL_LENGTH     |                              |
                         |                     _|                              |
                                   ZERO         --------------------------------
                                                                |
                                                              width
  * </pre>
*/
@Value.Immutable
@Value.Style(
        typeAbstract = "*Abs",
        typeImmutable = "*",
        get = ""
)
public abstract class StaticDataAbs {
  @Builder.Parameter
  public abstract int N();

  @Value.Default
  public int realN() {
    return N();
  }

  @Builder.Parameter
  public abstract double width();

  @Builder.Parameter
  public abstract double length();

  @Builder.Parameter
  public abstract double diameterOpening();

  @Value.Derived
  public double minDiameter() {
    return diameterOpening() == 0 ?  width() / 21 : diameterOpening() / 7;
  }

  @Value.Derived
  public double maxDiameter() {
    return diameterOpening() == 0 ?  width() / 15 : diameterOpening() / 5;
  }

  @Builder.Parameter
  public abstract double mass();

  @Builder.Parameter
  public abstract double kn();

  @Builder.Parameter
  public abstract double kt();

  @Value.Default
  public double simulationTime() {
    return 0;
  }

  @Value.Default
  public double delta1() {
    return 0;
  }

  @Value.Default
  public double delta2() {
    return 0;
  }

  @Value.Derived
  public double respawnMinY() {
    return fallLength() + length();
  }

  @Value.Derived
  public double respawnMaxY() {
    return respawnMinY() + maxDiameter();
  }

  @Value.Derived
  public double fallLength() {
    return length()/10; // Length of the area where particles fall out of the silo
  }

  @Value.Derived
  public double respawnLength() {
    return (respawnMaxY() - respawnMinY()); // Length of the area where particles respawn
  }

  @Value.Derived
  public double totalSystemLength() {
    return fallLength() + length() + respawnLength();
  }

  @Value.Default
  public boolean printOvito() {
    return false;
  }
}
