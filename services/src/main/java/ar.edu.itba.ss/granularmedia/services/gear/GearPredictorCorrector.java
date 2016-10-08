package ar.edu.itba.ss.granularmedia.services.gear;

import ar.edu.itba.ss.granularmedia.interfaces.NumericIntegrationMethod;
import ar.edu.itba.ss.granularmedia.models.Particle;
import ar.edu.itba.ss.granularmedia.models.Vector2D;

import java.util.Collection;
import java.util.HashSet;

import static java.lang.Math.pow;

public class GearPredictorCorrector<K extends GearSystemData> implements NumericIntegrationMethod<K> {
  @Override
  public void evolveSystem(final GearSystemData systemData,
                           final double dt) {
    predict(systemData, dt);
    evaluate(systemData, dt);
    fix(systemData, dt);
  }

  // private methods

  private void predict(final GearSystemData systemData, final double dt) {
    systemData.prePredict();

    final Collection<Particle> systemParticles = systemData.particles();
    final Collection<Particle> predictedSystemParticles = new HashSet<>();
    for (final Particle cSystemParticle : systemParticles) {
      predict(systemData, dt, cSystemParticle);
      predictedSystemParticles.add(predictedSystemParticle(systemData, cSystemParticle));
    }
    systemData.particles(predictedSystemParticles);

    systemData.postPredict();
  }

  private Particle predictedSystemParticle(final GearSystemData systemData, final Particle cSystemParticle) {
    final Vector2D uP = systemData.getPredictedR(cSystemParticle, 0);
    final Vector2D uV = systemData.getPredictedR(cSystemParticle, 1);
    final Vector2D uF = systemData.getPredictedR(cSystemParticle, 2).times(cSystemParticle.mass());
    return cSystemParticle.update(uP, uV, uF);
  }

  /*
    Bear in mind this predictor formulas of the GearPredictorCorrector prediction step:

    ---------------------------------------------------------
    r^(p)_k(t+Δt) = Σ_((i=k;term=0) to (ORDER; term=ORDER-k)) (r_i(t) * ((Δt)^(term) / term!)),

    with k the order of the derivative being calculated,
    term the current term of the
    'p' meaning to 'predicted' value

    Variables within this method associated with the above formula
    cTermDerivativeOrder := i
    cTermDerivativeValue := r_i(t)
    cTermConstantValue := (Δt)^(term) / term!
    ---------------------------------------------------------
   */
  private void predict(final GearSystemData systemData, final double dt, final Particle particle) {
    // we are going to update the values of all the orders of the derivative for this particle
    // we start iterating over all orders, from the first to the last
    for (int cDerivativeOrder = 0; cDerivativeOrder <= systemData.order() ; cDerivativeOrder++) {
      // initialize the new derivative value;
      // we are going to calculate this new value by adding one term at a time
      Vector2D cUpdatedDerivativeValue = Vector2D.builder(0,0).build();
      // watching the gear's prediction step, we can notice that for each new derivative value,
      // the number of terms required is the total order - the current derivative order + 1
      final int nRequiredTerms = systemData.order() - cDerivativeOrder + 1;

      // now, as we said, we are going to calculate each term individually, one at a time,
      // and append it to the new derivative value
      for (int cTerm = 0 ; cTerm < nRequiredTerms ; cTerm ++) {
        // we get the order of the derivative involved in the term being calculated
        // again, this can be checked by comparing with any of the derivative formulas
        // at the gear's prediction step
        final int cTermDerivativeOrder = cDerivativeOrder + cTerm;
        // we get the value of the derivative with the above order of the current particle
        final Vector2D cTermDerivativeValue = systemData.getR(particle, cTermDerivativeOrder);
        // we get the constant value that goes along with the current term being calculated
        final double cTermConstantValue = pow(dt, cTerm) / systemData.factorial(cTerm);
        // we append this term to all the previous terms
        cUpdatedDerivativeValue = cUpdatedDerivativeValue.add(cTermDerivativeValue.times(cTermConstantValue));
        // we go on with the next term, if any
      }

      // we've just calculated the updated value of the derivative with order cDerivativeOrder
      // let's update its value on the system's data
      systemData.setPredictedR(particle, cDerivativeOrder, cUpdatedDerivativeValue);

      // well, we are done with this order of derivation, let's go on with the next one, if any
    }
  }

  private void evaluate(final GearSystemData systemData, final double dt) {
    systemData.preEvaluate();

    final Collection<Particle> systemParticles = systemData.particles();
    for (final Particle cSystemParticle : systemParticles) {
      evaluate(systemData, dt, cSystemParticle);
    }

    systemData.postEvaluate();
  }

  private void evaluate(final GearSystemData systemData,
                        final double dt,
                        final Particle particle) {
    final Vector2D accelerationWithPredictedVariables = systemData.getForceWithPredicted(particle).div(particle.mass());
    // '2¡ value taken from Gear Predictor Corrector theory
    final Vector2D predictedAcceleration = systemData.getPredictedR(particle, 2);
    final Vector2D deltaAcceleration = accelerationWithPredictedVariables.sub(predictedAcceleration);
    final double constant = pow(dt, 2)/systemData.factorial(2); // taken from Gear Predictor Corrector theory
    final Vector2D deltaR2 = deltaAcceleration.times(constant);
    systemData.setDeltaR2(particle, deltaR2);
  }

  private void fix(final GearSystemData systemData, final double dt) {
    systemData.preFix();

    final Collection<Particle> systemParticles = systemData.particles();
    final Collection<Particle> updatedSystemParticles = new HashSet<>(systemParticles.size());
    for (final Particle cSystemParticle : systemParticles) {
      fix(systemData, dt, cSystemParticle);
      // update system's particle
      updatedSystemParticles.add(updatedParticle(systemData, cSystemParticle));
    }
    // update all system's particles
    systemData.particles(updatedSystemParticles);

    systemData.postFix();
  }

  private void fix(final GearSystemData systemData, final double dt, final Particle particle) {
    for(int cDerivativeOrder = 0; cDerivativeOrder <= systemData.order() ; cDerivativeOrder++) {
      final Vector2D predictedR = systemData.getPredictedR(particle, cDerivativeOrder);
      final Vector2D deltaR2 = systemData.getDeltaR2(particle);
      final double constant = systemData.alpha(cDerivativeOrder) * systemData.factorial(cDerivativeOrder) / pow(dt, cDerivativeOrder);
      final Vector2D secondTerm = deltaR2.times(constant);
      final Vector2D updatedR = predictedR.add(secondTerm);
      systemData.setR(particle, cDerivativeOrder, updatedR);
    }
  }

  private Particle updatedParticle(final GearSystemData systemData, final Particle particle) {
    final Vector2D uPosition = systemData.getR(particle, 0);
    final Vector2D uVelocity = systemData.getR(particle, 1);
    final Vector2D uForce = systemData.getR(particle, 2).times(particle.mass());
    return particle.update(uPosition, uVelocity, uForce);
  }
}
