package ar.edu.itba.ss.granularmedia.services.gear;


import ar.edu.itba.ss.granularmedia.interfaces.SystemData;
import ar.edu.itba.ss.granularmedia.models.Particle;
import ar.edu.itba.ss.granularmedia.models.Vector2D;

import java.util.*;

import static java.lang.Math.pow;

/* package-private */ abstract class GearSystemData implements SystemData {
    /*
      General notes:
        - R stands for position vector
        - Map notation
          - #key: value, where value can be a nested map with the notation { #key: value }
   */

  /**
   * Map saving each particle predicted derivatives, at different orders.
   * Map usage:
   *  #particle => { #derivative_order => derivative_value }
   */
  private final Map<Particle, Map<Integer, Vector2D>> predictedRs;

  /**
   * Map saving each particle current derivatives, at different orders.
   * Map usage:
   *  #particle => { #derivative_order => derivative_value }
   */
  private final Map<Particle, Map<Integer, Vector2D>> currentRs;

  /**
   * Map saving each particle current derivatives, at different orders.
   * Map usage:
   *  #particle => delta_R2_value }
   */
  private final Map<Particle, Vector2D> deltasR2;
  private final Map<Integer, Map<Double, Double>> predictedConstants;
  private final Map<Integer, Map<Double, Double>> fixConstants;
  private final Map<Double, Double> evaluateConstants;
  /* init */ {
    predictedConstants = new HashMap<>();
    fixConstants = new HashMap<>();
    for (int order = 0 ; order < sVectors() ; order++) {
      predictedConstants.put(order, new HashMap<>());
      fixConstants.put(order, new HashMap<>());
    }
    evaluateConstants = new HashMap<>();
  }

  /**
   * System's particles
   */
  private Collection<Particle> particles;
  private Collection<Particle> predictedParticles;

  /* package-private */ GearSystemData(final Collection<Particle> particles) {
    this.particles = particles;
    this.predictedParticles = new HashSet<>();

    final int nParticles = particles.size();

    this.predictedRs = new HashMap<>(nParticles);
    this.currentRs = new HashMap<>(nParticles);
    this.deltasR2 = new HashMap<>(nParticles);
  }

  @Override
  public Collection<Particle> particles() {
    return particles;
  }

  // protected
  // access from package and subclasses of any package

  /**
   * Sets the initial derivative values of the current system's particle.
   * The amount of derivative values calculated should match the order of the implemented system's data manager.
   * For example, for a Gear of order 5 implementation, the return map size should be 6, containing
   * derivative values from order 0 to order 5.
   * <P>
   * Also, take into account that implementation should be provided accordingly to the system
   * that needs to be simulated.
   * @param particle the particle whose derivative values are going to be calculated
   * @return a map containing all the derivative values from order 0 to the corresponding order, accordingly
   * to the GearSystemData's manager being implemented
   * @implNote
   * - if there is the need to use the rest of the system's particles just call particles() method
   */
  protected abstract Map<Integer, Vector2D> setInitialDerivativeValues(final Particle particle);

  /**
   * Calculates the current's particle force among the system.
   * <P>
   * Take into account that implementation should be provided accordingly to the system
   * that needs to be simulated.
   * @param particle the particle whose force with the predicted values is going to be calculated
   * @return a force vector calculated with the predicted values
   * @implNote
   * - if there is the need to use the rest of the system's particles just call particles() method
   */
  protected abstract Vector2D getForceWithPredicted(final Particle particle);


  // default implemented methods

  /**
   * Initialize the R and predictedR system values for this system's particle
   * using the provided {@code setInitialDerivativeValues} method on each particle.
   *
   * @implNote <b>Important: </b> This method should be called only once FOR EACH PARTICLE
   * before the usage of the gear method
   */
  protected void initParticle(final Particle particle) {
    // initialize maps
    // create internal predicted map
    this.predictedRs.put(particle, new HashMap<>(sVectors()));
    // initialize currentRs map with the first step, using the formula provided for this system
    this.currentRs.put(particle, setInitialDerivativeValues(particle));
  }

  /**
   *
   * @return the order of the data manager
   */
  protected abstract int order();

  /**
   * @return the size of the memory structures to be used when using this data manager
   */
  protected abstract int sVectors();

  /**
   * Gets the predicted R value of order {@code derivativeOrder} of the given {@code particle}
   * @param particle the particle whose predicted R value of order {@code derivativeOrder} wants to be retrieved
   * @param derivativeOrder the order of the R value to be retrieved for the given particle
   * @return the predicted R value of order {@code derivativeOrder} of the given {@code particle}; null if none
   */
  protected Vector2D getPredictedR(final Particle particle, final int derivativeOrder) {
    return predictedRs.get(particle).get(derivativeOrder);
  }

  /**
   * Execute some statements before prediction step
   */
  @SuppressWarnings("WeakerAccess")
  protected void prePredict() {

  }

  /**
   * Execute some statements just after prediction step for the given particle
   */
  @SuppressWarnings("WeakerAccess")
  public void predicted(@SuppressWarnings("UnusedParameters") final Particle predictedParticle) {

  }

  /**
   * Execute some statements after prediction step
   */
  @SuppressWarnings("WeakerAccess")
  protected void postPredict() {

  }

  /**
   * Execute some statements before evaluate step
   */
  @SuppressWarnings("WeakerAccess")
  protected void preEvaluate() {

  }

  /**
   * Execute some statements after evaluate step
   */
  @SuppressWarnings({"WeakerAccess", "unused"})
  protected void postEvaluate() {

  }

  /**
   * Execute some statements before fix step
   */
  @SuppressWarnings("WeakerAccess")
  protected void preFix() {

  }

  /**
   * Execute some statements after evaluate step
   */
  @SuppressWarnings("WeakerAccess")
  protected void postFix() {

  }

  /* package-private */
  protected Collection<Particle> predictedParticles() {
    return this.predictedParticles;
  }

  // package-private
  // access allowed from this module and gear package only, i.e., from Gear implementations only

  /**
   * Retrieves the constant for the predicted step of the specified term and the given dt
   * @param cTerm the term whose constant is needed
   * @param dt the dt used to calculate the constant
   * @return the constant for the predicted step for the specified parameters
   */
  /* package-private */ double getPredictedConstantTerm(final int cTerm, final double dt) {
    return predictedConstants.get(cTerm).computeIfAbsent(dt, aDouble -> pow(dt, cTerm) / factorial(cTerm));
  }

  /* package-private */ double getEvaluateConstant(final double dt) {
    // taken from Gear Predictor Corrector theory
    return evaluateConstants.computeIfAbsent(dt, aDouble -> pow(dt, 2)/factorial(2));
  }

  /* package-private */ double getFixConstantOrder(final int order, final double dt) {
    return fixConstants.get(order).computeIfAbsent(dt,
            aDouble -> alpha(order) * factorial(order) / pow(dt, order));
  }

  /* package-private */ void predictedParticles(final Collection<Particle> predictedParticles) {
    this.predictedParticles = predictedParticles;
  }

  /* package-private */ int nParticles() {
    return particles.size();
  }

  /**
   * Sets the new predicted R value - {@code updatedR} - of order {@code derivativeOrder} of the given {@code particle}
   * @param particle the particle whose predicted R value of order {@code derivativeOrder} wants to be set
   * @param derivativeOrder the order of the predicted R value to be set for the given {@code particle}
   * @param updatedR the updated predicted R value
   * @return the previous predicted R value; null if none
   */
  /* package-private */ Vector2D setPredictedR(final Particle particle,
                                               final int derivativeOrder,
                                               final Vector2D updatedR) {
    return predictedRs.get(particle).put(derivativeOrder, updatedR);
  }

  /**
   * Gets the R value of order {@code derivativeOrder} of the given {@code particle}
   * @param particle the particle whose R value of order {@code derivativeOrder} wants to be retrieved
   * @param derivativeOrder the order of the R value to be retrieved for the given particle
   * @return the R value of order {@code derivativeOrder} of the given {@code particle}; null if none
   */
  /* package-private */ Vector2D getR(final Particle particle, final int derivativeOrder) {
    return currentRs.get(particle).get(derivativeOrder);
  }

  /**
   * Sets the new R value - {@code updatedR} - of order {@code derivativeOrder} of the given {@code particle}
   * @param particle the particle whose R value of order {@code derivativeOrder} wants to be set
   * @param derivativeOrder the order of the R value to be set for the given {@code particle}
   * @param updatedR the updated R value
   * @return the previous R value; null if none
   */
  /* package-private */ Vector2D setR(final Particle particle, final int derivativeOrder, final Vector2D updatedR) {
    return currentRs.get(particle).put(derivativeOrder, updatedR);
  }

  /**
   * Gets the {@code deltaR2} value of the given {@code particle}
   * @param particle the particle whose {@code deltaR2} value wants to be retrieved
   * @return the {@code deltaR2} value of the given {@code particle}; null if none
   */
  /* package-private */ Vector2D getDeltaR2(final Particle particle) {
    return deltasR2.get(particle);
  }

  /**
   * Sets the new {@code deltaR2} value - {@code updatedDeltaR2} - of the given {@code particle}
   * @param particle the particle whose {@code deltaR2} value wants to be set
   * @return the previous {@code deltaR2} value; null if none
   */
  /* package-private */ Vector2D setDeltaR2(final Particle particle, final Vector2D updatedDeltaR2) {
    return deltasR2.put(particle, updatedDeltaR2);
  }

  /**
   * Updates all the current system's particles
   * @param updatedSystemParticles the new system's particles
   * @implNote This method should be used to update system's particle status only
   */
  /* package-private */ void particles(final Collection<Particle> updatedSystemParticles) {
    particles = updatedSystemParticles;
  }

  /**
   * Retrieves the precomputed factorial of {@code n}
   * @param n number to be used
   * @return the precomputed factorial of {@code n}
   * @implNote n should be <= order() as there is no need to use greater values
   * in any Gear Predict Corrector implementations
   */
  /* package-private */ abstract long factorial(final int n);

  /**
   * Retrieves the precomputed factorial of {@code n}
   * @param n number to be used
   * @return the precomputed factorial of {@code n}
   * @implNote n should be <= order() as there is no need to use greater values
   * in any Gear Predict Corrector implementations
   */
  /* package-private */ abstract double alpha(final int n);

  /**
   * Computes the factorial of {@code n}
   * @param n number whose factorial will be computed
   * @return the factorial of {@code n}
   */
  /* package-private */ static long staticFactorial(final int n) {
    int number = n;
    long factorial = 1;
    while ( number > 0) {
      factorial *= number;
      number--;
    }

    return factorial;
  }
}
