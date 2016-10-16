package ar.edu.itba.ss.granularmedia.interfaces;

/* package-private */ interface SimulationSystem<E extends SystemData> {
  /**
   *
   * @return the being simulated system's data
   */
  E getSystemData();
}
