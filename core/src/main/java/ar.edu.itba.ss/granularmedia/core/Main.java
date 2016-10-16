package ar.edu.itba.ss.granularmedia.core;

import ar.edu.itba.ss.granularmedia.core.system.GenerateDynamicFileProgram;
import ar.edu.itba.ss.granularmedia.core.system.GenerateStaticFileProgram;
import ar.edu.itba.ss.granularmedia.core.system.GranularMediaSystemProgram;
import ar.edu.itba.ss.granularmedia.core.system.HelpProgram;
import ar.edu.itba.ss.granularmedia.interfaces.MainProgram;
import ar.edu.itba.ss.granularmedia.services.IOService;

public class Main {
  public static void main(String[] args) {
    if (args.length == 0) {
      IOService.exit(IOService.ExitStatus.NO_ARGS, null);
    }
    final MainProgram mainProgram;
    switch (args[0]) {
      case "help":
        mainProgram = new HelpProgram();
        break;
      case "gen":
        mainProgram = chooseGenerator(args);
        break;
      case "sim":
        mainProgram = new GranularMediaSystemProgram();
        break;
      default:
        IOService.exit(IOService.ExitStatus.BAD_ARGUMENT, null);
        return;
    }
    mainProgram.run(args);
  }

  private static MainProgram chooseGenerator(final String[] args) {
    if (args.length < 1) {
      IOService.exit(IOService.ExitStatus.BAD_N_ARGUMENTS, null);
      // should never return from the above code
      throw new IllegalStateException();
    }

    switch (args[1]) {
      case "static":
        return new GenerateStaticFileProgram();
      case "dynamic":
        return new GenerateDynamicFileProgram();
      default:
        IOService.exit(IOService.ExitStatus.BAD_ARGUMENT, null);
        // should never return from the above code
        throw new IllegalStateException();
    }
  }
}
