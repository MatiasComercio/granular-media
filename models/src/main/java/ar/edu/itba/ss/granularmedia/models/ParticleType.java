package ar.edu.itba.ss.granularmedia.models;

public enum ParticleType {
  OPENING_LEFT(1),
  OPENING_RIGHT(2),
  SPAWN(3),
  WALL(4),
  COMMON(5),
  BORDER(6);

  private final int code;

  ParticleType(final int code) {
    this.code = code;
  }

  public int getCode() {
    return code;
  }
}
