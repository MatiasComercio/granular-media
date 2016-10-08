package ar.edu.itba.ss.granularmedia.core;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import ar.edu.itba.ss.granularmedia.interfaces.Interface;
import ar.edu.itba.ss.granularmedia.services.Service;

public class Main {
  private static final Logger LOGGER = LoggerFactory.getLogger(Main.class);

  public static void main(String[] args) {
    LOGGER.info("App running...");
    final Interface i = new Service();
    i.printToScreen();
    LOGGER.info("App ending...");
  }
}
