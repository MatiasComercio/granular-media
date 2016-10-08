package ar.edu.itba.ss.granularmedia.core;

import ar.edu.itba.ss.granularmedia.core.system.GranularMediaSystemProgram;
import ar.edu.itba.ss.granularmedia.interfaces.MainProgram;

public class Main {
  public static void main(String[] args) {
    final MainProgram granularSystem = new GranularMediaSystemProgram();
    granularSystem.run(args);
  }
}
