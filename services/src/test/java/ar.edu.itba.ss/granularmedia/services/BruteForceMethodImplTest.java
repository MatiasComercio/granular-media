package ar.edu.itba.ss.granularmedia.services;

import ar.edu.itba.ss.granularmedia.interfaces.NeighboursFinder;
import ar.edu.itba.ss.granularmedia.models.Particle;
import ar.edu.itba.ss.granularmedia.services.neighboursfinders.BruteForceMethodImpl;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import java.util.*;

public class BruteForceMethodImplTest {
    private final NeighboursFinder bruteForceMethod;

    public BruteForceMethodImplTest() {
        bruteForceMethod = new BruteForceMethodImpl();
    }

    @Test
    public void runWithNoPeriodicLimitTest() {
        final double r = 0.5;

        final Set<Particle> points = new HashSet<>();
        final Particle p1 = Particle.builder(1,1).radio(r).build();
        final Particle p2 = Particle.builder(2.5,1).radio(r).build();
        final Particle p3 = Particle.builder(4,1).radio(r).build();
        final Particle p4 = Particle.builder(2.5,2.5).radio(r).build();
        final Particle p5 = Particle.builder(4,5.5).radio(r).build();

        points.add(p1);
        points.add(p2);
        points.add(p3);
        points.add(p4);
        points.add(p5);

        final double L = 6d;
<<<<<<< HEAD
        final double W = 6d;
=======
>>>>>>> 277d289... Added - our changes
        final int M = 2;
        final double rc = 1.5;
        final boolean periodicLimit = false;

<<<<<<< HEAD
        final Map<Particle, Collection<Particle>> processedPoints = bruteForceMethod.run(points, L, W, M, M, rc, periodicLimit);
=======
        final Map<Particle, Collection<Particle>> processedPoints = bruteForceMethod.run(points, L, M, rc, periodicLimit);
>>>>>>> 277d289... Added - our changes

        final Map<Particle, Collection<Particle>> expectedProcessedPoints = new HashMap<>();

        // p1
        final Set<Particle> neighbours1 = new HashSet<>();
        neighbours1.add(p2);
        neighbours1.add(p4);
        expectedProcessedPoints.put(p1, neighbours1);

        // p2
        final Set<Particle> neighbours2 = new HashSet<>();
        neighbours2.add(p1);
        neighbours2.add(p3);
        neighbours2.add(p4);
        expectedProcessedPoints.put(p2, neighbours2);

        // p3
        final Set<Particle> neighbours3 = new HashSet<>();
        neighbours3.add(p2);
        neighbours3.add(p4);
        expectedProcessedPoints.put(p3, neighbours3);

        // p4
        final Set<Particle> neighbours4 = new HashSet<>();
        neighbours4.add(p1);
        neighbours4.add(p2);
        neighbours4.add(p3);
        expectedProcessedPoints.put(p4, neighbours4);

        // p5
        final Set<Particle> neighbours5 = new HashSet<>();
        expectedProcessedPoints.put(p5, neighbours5);

        Assert.assertEquals(expectedProcessedPoints, processedPoints);
    }

}