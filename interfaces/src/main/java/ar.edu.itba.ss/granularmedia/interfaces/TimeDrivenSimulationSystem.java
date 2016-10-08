package ar.edu.itba.ss.granularmedia.interfaces;

public interface TimeDrivenSimulationSystem extends SimulationSystem {
  /**
   * Evolves the system a {@code dt} time interval.
   *
   * @param dt the time step used to evolve the system
   */
  void evolveSystem(double dt);
}
