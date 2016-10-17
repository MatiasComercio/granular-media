package ar.edu.itba.ss.granularmedia.services;

import ar.edu.itba.ss.granularmedia.interfaces.NeighboursFinder;
import ar.edu.itba.ss.granularmedia.models.Particle;
import ar.edu.itba.ss.granularmedia.services.neighboursfinders.CellIndexMethodImpl;
import org.junit.Assert;
import org.junit.Test;

import java.util.*;

public class CellIndexMethodImplTest {

  private final NeighboursFinder nonPeriodicCellIndexMethod;
  private final NeighboursFinder periodicCellIndexMethod;

  public CellIndexMethodImplTest() {
    final double L = 6d;
    final double W = 6d;
    final int M1 = 2;
    final int M2 = 2;
    final double rc = 1.5;
    nonPeriodicCellIndexMethod = new CellIndexMethodImpl(L, W, M1, M2, rc, false);
    periodicCellIndexMethod = new CellIndexMethodImpl(L, W, M1, M2, rc, true);
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

    final Map<Particle, Collection<Particle>> processedPoints = nonPeriodicCellIndexMethod.run(points);

    final Map<Particle, Set<Particle>> expectedProcessedPoints = new HashMap<>();

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
    expectedProcessedPoints.put(p5, new HashSet<>());

    Assert.assertEquals(expectedProcessedPoints, processedPoints);
  }

  @Test
  public void runWithPeriodicLimitTest() {
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

    final Map<Particle, Collection<Particle>> processedPoints = periodicCellIndexMethod.run(points);

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
    neighbours2.add(p5);
    expectedProcessedPoints.put(p2, neighbours2);

    // p3
    final Set<Particle> neighbours3 = new HashSet<>();
    neighbours3.add(p2);
    neighbours3.add(p4);
    neighbours3.add(p5);
    expectedProcessedPoints.put(p3, neighbours3);

    // p4
    final Set<Particle> neighbours4 = new HashSet<>();
    neighbours4.add(p1);
    neighbours4.add(p2);
    neighbours4.add(p3);
    expectedProcessedPoints.put(p4, neighbours4);

    // p5
    final Set<Particle> neighbours5 = new HashSet<>();
    neighbours5.add(p2);
    neighbours5.add(p3);
    expectedProcessedPoints.put(p5, neighbours5);

    Assert.assertEquals(expectedProcessedPoints, processedPoints);
  }

  @Test(expected=IndexOutOfBoundsException.class)
  public void testUpIndexOutOfBoundsException() {
    final double r = 0.5;
    final Set<Particle> points = new HashSet<>();
    final Particle p1 = Particle.builder(6, 6).radio(r).build(); // this point should fail when run is called  (out of bounds)
    points.add(p1);

    periodicCellIndexMethod.run(points);
  }

  @Test(expected=IndexOutOfBoundsException.class)
  public void testDownIndexOutOfBoundsException() {
    final double r = 0.5;
    final Set<Particle> points = new HashSet<>();
    final Particle p1 = Particle.builder(-0.5, 5).radio(r).build(); // this point should fail when run is called  (out of bounds)
    points.add(p1);

    periodicCellIndexMethod.run(points);
  }

  @Test
  public void runWithLimitCases() {
    final double r = 0.5;

    final Set<Particle> points = new HashSet<>();
    final Particle p1 = Particle.builder(1,1).radio(r).build();
    final Particle p2 = Particle.builder(2.5,1).radio(r).build();
    final Particle p3 = Particle.builder(4,1).radio(r).build();
    final Particle p4 = Particle.builder(2.5,2.5).radio(r).build();
    final Particle p5 = Particle.builder(4,5.5).radio(r).build();
    final Particle p6 = Particle.builder(5.5,5.5).radio(r).build();
    final Particle p7 = Particle.builder(0.5,5.5).radio(r).build();
    final Particle p8 = Particle.builder(5.5,0.5).radio(r).build();

    points.add(p1);
    points.add(p2);
    points.add(p3);
    points.add(p4);
    points.add(p5);
    points.add(p6);
    points.add(p7);
    points.add(p8);

    Map<Particle, Collection<Particle>> processedPoints;

    processedPoints = periodicCellIndexMethod.run(points);

    final Map<Particle, Set<Particle>> expectedProcessedPoints = new HashMap<>();

    // p1
    final Set<Particle> neighbours1 = new HashSet<>();
    neighbours1.add(p2);
    neighbours1.add(p4);
    neighbours1.add(p6);
    neighbours1.add(p7);
    neighbours1.add(p8);
    expectedProcessedPoints.put(p1, neighbours1);

    // p2
    final Set<Particle> neighbours2 = new HashSet<>();
    neighbours2.add(p1);
    neighbours2.add(p3);
    neighbours2.add(p4);
    neighbours2.add(p5);
    neighbours2.add(p7);
    expectedProcessedPoints.put(p2, neighbours2);

    // p3
    final Set<Particle> neighbours3 = new HashSet<>();
    neighbours3.add(p2);
    neighbours3.add(p4);
    neighbours3.add(p5);
    neighbours3.add(p6);
    neighbours3.add(p8);
    expectedProcessedPoints.put(p3, neighbours3);

    // p4
    final Set<Particle> neighbours4 = new HashSet<>();
    neighbours4.add(p1);
    neighbours4.add(p2);
    neighbours4.add(p3);
    expectedProcessedPoints.put(p4, neighbours4);

    // p5
    final Set<Particle> neighbours5 = new HashSet<>();
    neighbours5.add(p2);
    neighbours5.add(p3);
    neighbours5.add(p6);
    neighbours5.add(p7);
    neighbours5.add(p8);
    expectedProcessedPoints.put(p5, neighbours5);

    // p6
    final Set<Particle> neighbours6 = new HashSet<>();
    neighbours6.add(p1);
    neighbours6.add(p3);
    neighbours6.add(p5);
    neighbours6.add(p7);
    neighbours6.add(p8);
    expectedProcessedPoints.put(p6, neighbours6);

    // p7
    final Set<Particle> neighbours7 = new HashSet<>();
    neighbours7.add(p1);
    neighbours7.add(p2);
    neighbours7.add(p5);
    neighbours7.add(p6);
    neighbours7.add(p8);
    expectedProcessedPoints.put(p7, neighbours7);

    // p8
    final Set<Particle> neighbours8 = new HashSet<>();
    neighbours8.add(p1);
    neighbours8.add(p3);
    neighbours8.add(p5);
    neighbours8.add(p6);
    neighbours8.add(p7);
    expectedProcessedPoints.put(p8, neighbours8);

    Assert.assertEquals(expectedProcessedPoints, processedPoints);

    // print debug to help a better analysis
//  expectedProcessedPoints.forEach((point, neighbours) -> {
//   System.out.println(point);
//   System.out.println("Expected Neighbours: ");
//   neighbours.forEach(System.out::println);
//   System.out.println();
//   final Set<Particle> processedNeighbours = processedPoints.get(point);
//   System.out.println("Processed Neighbours: ");
//   processedNeighbours.forEach(System.out::println);
//   System.out.println();
//   System.out.println("------------------------------");
//   System.out.println();
//  });
  }

}

