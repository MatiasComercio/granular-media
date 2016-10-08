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
}
