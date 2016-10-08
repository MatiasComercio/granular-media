package ar.edu.itba.ss.granularmedia.models;

import org.immutables.builder.Builder;
import org.immutables.value.Value;
import org.omg.CORBA.DoubleHolder;

@Value.Immutable
@Value.Style(
        typeAbstract = "*Abs",
        typeImmutable = "*",
        get = ""
)
public abstract class Vector2DAbs {
  @Builder.Parameter
  public abstract double x();

  @Builder.Parameter
  public abstract double y();

  @Value.Derived
  @Value.Auxiliary
  public double norm2() {
    return Math.sqrt(Math.pow(x(),2) + Math.pow(y(),2));
  }

  public Vector2D add(final Vector2DAbs v) {
    return Vector2D.builder(x() + v.x(), y() + v.y()).build();
  }
  public Vector2D sub(final Vector2DAbs v) {
    return Vector2D.builder(x() - v.x(), y() - v.y()).build();
  }

  public Vector2D times(final double c) {
    return Vector2D.builder(x() * c, y() * c).build();
  }

  public Vector2D div(final double c) {
    return Vector2D.builder(x() / c, y() / c).build();
  }

  public Vector2D toVersor() {
    final double norm2 = norm2();
    if (Double.valueOf(0).equals(norm2)) {
      return null;
    }
    return Vector2D.builder(x()/norm2, y()/norm2).build();
  }
}
