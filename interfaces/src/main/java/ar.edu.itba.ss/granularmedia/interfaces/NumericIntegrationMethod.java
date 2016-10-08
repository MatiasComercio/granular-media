package ar.edu.itba.ss.granularmedia.interfaces;

public interface NumericIntegrationMethod<K> {

  /**
   * Evolves the system described within the system's data a {@code dt} time interval.
   * System's evolution is based on and saved at the given {@code systemData}.
   * <P>
   * All the necessary system's logic to be used by the numeric method should be contained inside the {@code systemData}
   * <P>
   * One can defined the {@code systemData} depending on the system and accordingly to the numeric integration
   * method to be used.
   * <P>
   * For example, consider an Oscillator System to be solved with the Gear Integration Method
   * <pre>
   * {@code
   *  public class GearOscillatorSystem implements TimeDrivenSimulationSystem, Oscillator {
   *
   *    private final NumericIntegrationMethod<Gear5SystemData> numericIntegrationMethod;
   *    private final Gear5SystemData systemData;
   *
   *    public GearOscillatorSystem(final double mass, final double r, final double k, final double gamma) {
   *      // instantiate all needed particles
   *      // create system data container with system's particles and constants
   *
   *      // Numeric Integration Method initialization
   *      this.systemData = new OscillatorGear5SystemData(particles, k, gamma); // subclass of Gear5SystemData
   *      this.numericIntegrationMethod = new GearPredictorCorrector<>();
   *    }
   *  }
   * }
   * </pre>
   * @param systemData entity that will manage numeric method access to the system.
   *                   This class should be defined as a mediator between the system's logic and the numeric integration
   *                   method's logic.
   * @param dt the time step used to evolve the system
   */
  void evolveSystem(final K systemData,
                    final double dt);
}
