package ar.edu.itba.ss.granularmedia.services.neighboursfinders;

import ar.edu.itba.ss.granularmedia.interfaces.NeighboursFinder;
import ar.edu.itba.ss.granularmedia.models.Particle;
import ar.edu.itba.ss.granularmedia.services.apis.Space2DMaths;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;

public class CellIndexMethodImpl implements NeighboursFinder {
  private static final Logger LOGGER = LoggerFactory.getLogger(CellIndexMethodImpl.class);
  /**
   * On the run method, given a cell (row, col), it is necessary to compare with itself
   * and to go up, up-right, right and down-right. This is:
   *    itself = row, col
   *    up = row-1, col
   *    up-right = row-1, col+1
   *    right = row, col+1
   *    down-right = row+1, col+1
   */
  private final static int[][] neighbourDirections = new int[][] {
          {0, 0}, // itself
          {-1, 0}, // up
          {-1, +1}, // up-right
          {0, +1}, // right
          {+1, +1} // down-right
  };
  private static final int ROW = 0;
  private static final int COL = 1;


  @Override
  public Map<Particle, Collection<Particle>> run(final Collection<Particle> particles, final double L, final double W, final int M1, final int M2, final double rc, final boolean periodicLimit) {
    // check M conditions

    if (M1 <= 0 || M2<=0 || rc < 0 || L <= 0 || W<=0) {
      throw new IllegalArgumentException("Check that this is happening, but must not: M1 <= 0 or M2<= 0 or rc < 0 or L <= 0 or W<=0");
    }

    // create the cell matrix (Size: M1 x M2)
    final CellMatrix cellMatrix = new CellMatrix(M1, M2);

    final Map<Particle, Collection<Particle>> collisionPerParticle = new HashMap<>(particles.size());
    final Set<Cell> nonEmptyCells = new HashSet<>();

    for (Particle point : particles) {
      // add the point to the map to be returned, with a new empty set
      collisionPerParticle.put(point, new HashSet<>());

      // put each point on the corresponding cell of the cell's matrix
      // save the cell as a non empty one, to analyse it later
      nonEmptyCells.add(saveToMatrix(L, W, M1, M2, point, cellMatrix));
    }

    // run the cell index method itself
    run(L, W, nonEmptyCells, cellMatrix, rc, periodicLimit, collisionPerParticle);

    // return the created map with each point information
    return collisionPerParticle;
  }

  private void run(final double L, final double W, final Set<Cell> nonEmptyCells, final CellMatrix cellMatrix, final double rc,
                   final boolean periodicLimit, final Map<Particle, Collection<Particle>> collisionPerParticle) {
  /*
   Takes one cell at a time and applies the patter saw in class to take advantage of the symmetry of the
    method. Let's explain it a little bit.

   Given a cell (row, col), it is necessary to go up, up-right, right and down-right. This is:
   * up = row-1, col
   * up-right = row-1, col+1
   * right = row, col+1
   * down-right = row+1, col+1

   Periodic Limit Cases

   if periodic limit is false
    if row-1 < 0 || row+1 = #rows || col+1 = #cols => do not consider that cell, with M = matrix.dimension()

   if periodic limit is true
    if row-1 < 0 => use #rows-1 and particles inside this cell should be applied an y offset of + L
    if row+1 = #rows => use 0 and particles inside this cell should be applied an y offset of - L
    if col+1 = #cols => use 0 and particles inside this cell should be applied an x offset of + W

    with L beign the rectangle's height and W the width

   */

    final int rows = cellMatrix.rows();
    final int cols = cellMatrix.cols();
    nonEmptyCells.forEach(cCell -> {
      boolean virtualParticleNeeded;
      double xOffset, yOffset;
      int row, col, oRow, oCol;
      Cell oCell;
      for (final int[] neighbourDirection : neighbourDirections) { // travel getting different neighbours
        virtualParticleNeeded = false;
        xOffset = 0;
        yOffset = 0;

        // get current cell's row & col
        row = cCell.row;
        col = cCell.col;

        // get the other cell's row & col
        oRow = row + neighbourDirection[ROW];
        oCol = col + neighbourDirection[COL];

        // adapt to periodicLimit condition
        if (!periodicLimit) {
          if (oRow < 0 || oRow == rows || oCol == cols) {
            continue; // do not consider this cell, because it does not exists
          }
        } else {
          // oRow condition
          if (oRow < 0) {
            oRow = rows - 1;
            virtualParticleNeeded = true;
            yOffset = L;
          } else if (oRow == rows) {
            oRow = 0;
            virtualParticleNeeded = true;
            yOffset = -L;
          }

          // oCol condition
          if (oCol == cols) {
            oCol = 0;
            virtualParticleNeeded = true;
            xOffset = W;
          }
        }

        oCell = cellMatrix.get(oRow, oCol);

        // checks if it is the same cell
        if (cCell.equals(oCell)) {
          // if so, check collisions only on the current cell, using an improvement of the brute force method
          checkCollisions(cCell, rc, collisionPerParticle);
        } else if (nonEmptyCells.contains(oCell)) {
          // so as not to create overhead; if empty => no necessary to process

          // if !empty => check the distance between each pair of particles on the current pair of cells,
          // and add the necessary mappings, if two particles collide
          checkCollisions(cCell, oCell, rc, collisionPerParticle, virtualParticleNeeded, xOffset, yOffset);
        }
      }
    });
  }

  private void checkCollisions(final Cell cCell, final double rc, final Map<Particle, Collection<Particle>> collisionPerParticle) {
    Particle[] points = new Particle[cCell.particles.size()];
    points = cCell.particles.toArray(points);
    Particle pi, pj;
    for (int i = 0 ; i < points.length ; i++) {
      pi = points[i];
      for (int j = i+1 ; j < points.length ; j++) {
        pj = points[j];
        checkCollision(pi,pj,rc, collisionPerParticle);
      }
    }
  }

  /**
   * Check, for each pair of particles on each cell (c = current, o = other), if they are colliding, i.e.,
   * if the distance between them is lower or equal to rc, considering their radios too.
   * <p>
   * If so, each of them are added to the other's collision set at the given map.
   * <p>
   * Notice that if a virtual point is needed, virtualParticleNeeded should be true, and xOffset and yOffset
   * should have the corresponding values to be applied to all the particles of the oCell.
   * A virtual point should be needed when a border case is reached and a periodic limit is being considered.
   * @param cCell current cell being analysed
   * @param oCell other cell whose particles will be compared to the cCell's particles
   * @param rc max distance to consider that two particles are colliding
   * @param collisionPerParticle map containing all the particles and a set with the already found colliding particles, for each
   * @param virtualParticleNeeded whether a virtual point is needed or not
   * @param xOffset x offset to be applied to all the oCell's particles when creating the new virtual point
   * @param yOffset y offset to be applied to all the oCell's particles when creating the new virtual point
   */
  private void checkCollisions(final Cell cCell, final Cell oCell, final double rc,
                               final Map<Particle, Collection<Particle>> collisionPerParticle,
                               final boolean virtualParticleNeeded,
                               final double xOffset, final double yOffset) {
    cCell.particles.forEach(cParticle -> oCell.particles.forEach(oParticle -> {
      Particle vParticle = oParticle;
      if (virtualParticleNeeded) {
        // a virtual point should be considered instead of the real one
        vParticle = Particle.builder(oParticle.x() + xOffset, oParticle.y() + yOffset)
                .radio(oParticle.radio()).build();
      }
      if (Space2DMaths.distanceBetween(cParticle, vParticle) <= rc // particles are colliding
              && !cParticle.equals(vParticle)) { // for the case of identity
        // add each one to the set of colliding particles of the other
        collisionPerParticle.get(cParticle).add(oParticle);
        collisionPerParticle.get(oParticle).add(cParticle);
      }
    }));
  }

  private static boolean checkCollision(final Particle pi, final Particle pj, final double rc,
                                        final Map<Particle, Collection<Particle>> collisionPerPoint) {
    if (Space2DMaths.distanceBetween(pi, pj) <= rc) {
      collisionPerPoint.get(pi).add(pj);
      collisionPerPoint.get(pj).add(pi);
      return true;
    }
    return false;
  }

  /**
   *
   * @param L -
   * @param W -
   * @param M1 -
   * @param M2 -
   * @param point -
   * @param cellMatrix -
   * @return cell where the given point was saved at the given SquareMatrix
   */
  private Cell saveToMatrix(final double L, final double W, final int M1, final int M2,
                            final Particle point, final CellMatrix cellMatrix) {
  /*
    Each point has an x & y component.
    To get at which cell of the matrix the point belongs, here it is the idea of what's done.
    Consider the case of a column:
    * check which is the number t that makes t*k <= point.x() < (t+1)*k (where k is the length of a cell)
    * if t is an integer, the column taken is t-1 (unless t = 0), as it would be the case that the point is
      at a cell boundary, and it can be classified in any of those.
    * if t is not an integer, the floor of t is taken as the column number

    The same goes for the row.

    Notes:
     * For translating particles to the correct matrix index, the following formula is used:
       column = ( nCells - 1 ) - t

       The problem is "drawn" following, with nCell = 5 for this example

        (x,y) plain with t
        indexes as they are got

       y
       |
       4
       3
       2
       1
       0 1 2 3 4 --> x

        matrix with
        translated t indexes
        0 1 2 3 4
       0
       1
       2
       3
       4

       Notice that the second form is the one needed to work with the matrix,
       so as to be able to make a more efficient process

     * rows are calculated with point.y() and cols with point.x()
     (see previous graphics for a better understanding).
   */

    final double k1 = L / M1; // the height of each cell
    final double k2 = W / M2; // the base of each cell

    final int row, col;

    row = (M1 - 1) - getT(k1, point.y());
    col = getT(k2, point.x());

    // if row or col is out of bounds => bad input was given ( x < 0 || x >= L || y < 0 || y >= L )
    //TODO: REMOVE, THIS IS DEBUG ONLY
    try{
      return cellMatrix.addToCell(row, col, point);
    } catch (IndexOutOfBoundsException e){
//      LOGGER.error("Particle Out of Bounds!");
//      LOGGER.error("row: " + row + " col: " + col + " M1: " + M1 + " M2: " + M2 + " x: " + point.x() + " y: " + point.y());
      return cellMatrix.get(0,0); // This cell is added to non-empty, no harm done
    }

  }

  /**
   *
   * @param k k
   * @param v v
   * @return Math.toIntExact(Math.round [Math.floor [v div k]]));
   *
   * @throws ArithmeticException if Math.round(Math.floor(v/k)) overflows an int
   */
  private int getT(final double k, final double v) {
    return Math.toIntExact(Math.round(Math.floor(v/k)));
  }


  private static class Cell {
    private final List<Particle> particles;
    private final int row;
    private final int col;

    private Cell(final int row, final int col) {
      this.particles = new LinkedList<>();
      this.row = row;
      this.col = col;
    }

    @Override
    public boolean equals(final Object o) {
      if (this == o) return true;
      if (!(o instanceof Cell)) return false;

      final Cell cell = (Cell) o;

      return row == cell.row && col == cell.col;

    }

    @Override
    public int hashCode() {
      int result = row;
      result = 31 * result + col;
      return result;
    }
  }

  private static class CellMatrix {
    private final Cell[][] matrix;
    private final int rows;
    private final int cols;


    private CellMatrix(final int rows, final int cols) {
      this.rows = rows;
      this.cols = cols;
      this.matrix = new Cell[rows][cols];
      for (int row = 0 ; row < rows ; row ++) {
        for (int col = 0 ; col < cols ; col ++) {
          matrix[row][col] = new Cell(row, col);
        }
      }
    }

    /**
     *
     * @param row row index
     * @param col col index
     * @return the cell contained at the specified row and col
     *
     * @throws IndexOutOfBoundsException if row or col is lower than 0 or equal or greater than matrix's dimension
     */
    private Cell get(final int row, final int col) {
      return matrix[row][col];
    }

    /**
     *
     * @param row row index
     * @param col col index
     * @param p point to be added to the cell specified by the given row and call
     * @return the Cell where the point was added
     *
     * @throws IndexOutOfBoundsException if row or col is lower than 0 or equal or greater than matrix's dimension
     */
    private Cell addToCell(final int row, final int col, final Particle p) {
      final Cell c = get(row, col);
      c.particles.add(p);
      return c;
    }

    private int rows() {
      return rows;
    }
    private int cols() { return cols; }

  }
}

