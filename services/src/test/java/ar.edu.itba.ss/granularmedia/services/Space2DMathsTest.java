package ar.edu.itba.ss.granularmedia.services;

import ar.edu.itba.ss.granularmedia.models.Vector2D;
import ar.edu.itba.ss.granularmedia.services.apis.Space2DMaths;
import org.junit.Assert;
import org.junit.Test;

public class Space2DMathsTest {
  private final static double XI = 3.77;
  private final static double XJ = 2.55;
  private final static double YI = 1;
  private final static double YJ = 0;

  private final static double XJI = XJ-XI;
  private final static double YJI = YJ-YI;
  private final static double DISTANCE_JI = Math.sqrt(Math.pow(XJI, 2) + Math.pow(YJI, 2));


  private final static Vector2D i = Vector2D.builder(XI, YI).build();
  private final static Vector2D j = Vector2D.builder(XJ, YJ).build();

  @Test
  public void normalAndTangentialVersorsValidTest() {
    final Vector2D[] actualArray = Space2DMaths.normalAndTangentialVersors(i, j);

    final Vector2D nV = Vector2D.builder(XJI / DISTANCE_JI, YJI / DISTANCE_JI).build();
    final Vector2D tV = Vector2D.builder(- nV.y(), nV.x()).build();
    final Vector2D[] expectedArray = new Vector2D[] { nV, tV };

    Assert.assertArrayEquals(expectedArray, actualArray);
  }

  @Test
  public void normalAndTangentialVersorsInvalidTest() {
    final Vector2D[] actualArray = Space2DMaths.normalAndTangentialVersors(i, i);

    Assert.assertNull(actualArray);
  }
}
