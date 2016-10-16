package ar.edu.itba.ss.granularmedia.models;

import org.immutables.builder.Builder;
import org.immutables.value.Value;

@Value.Immutable
@Value.Style(
        typeAbstract = "*Abs",
        typeImmutable = "*",
        get = ""
)
public abstract class StaticDataAbs {
  @Builder.Parameter
  public abstract int N();

  @Builder.Parameter
  public abstract double width();

  @Builder.Parameter
  public abstract double length();

  @Builder.Parameter
  public abstract double diameterOpening();

  @Value.Derived
  public double minDiameter() {
    return diameterOpening() / 7;
  }

  @Value.Derived
  public double maxDiameter() {
    return diameterOpening() / 5;
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
}
