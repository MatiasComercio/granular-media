package ar.edu.itba.ss.granularmedia.services;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

import static ar.edu.itba.ss.granularmedia.services.IOService.ExitStatus.*;

public class IOService {
  private static final Logger LOGGER = LoggerFactory.getLogger(IOService.class);
  private static final String ERROR_WRITE_FILE_LOGGER =
          "An unexpected IO Exception occurred while writing the file {}. Caused by: ";

  private static final String ABORTING = "\nCheck logs for more info.\nAborting...\n";
  private static final String NO_DETAIL = "[NO DETAIL GIVEN]";

  // Exit Codes
  public enum ExitStatus {
    NO_ARGS(-1, "", ""),
    NO_FILE(-2, "", ""),
    BAD_N_ARGUMENTS(-3, "", ""),
    BAD_ARGUMENT(-4, "", "[FAIL] - Invalid argument. Try 'help' for more information."),
    NOT_A_FILE(-5, "", ""),
    UNEXPECTED_ERROR(-6, "", ""),
    BAD_FILE_FORMAT(-7, "", ""),
    MKDIRS_FAILED(-8,
            "[FAIL] - Create directory operation failed while trying to create dir: '{}'",
            "[FAIL] - Create directory operation failed." + ABORTING),
    VALIDATION_FAILED(-9,
            "[FAIL] - Validation not passed: {}",
            "[FAIL] - Validation not passed." + ABORTING),
    DELETE_EXISTING_FILE_FAILED(-10,
            "[FAIL] - Could not delete the existing file: '{}'",
            "[FAIL] - Could not delete an existing file." + ABORTING);

    private final int code;
    private final String loggerMsg;
    private final String msg;

    ExitStatus(final int code, final String loggerMsg, final String msg) {
      this.code = code;
      this.loggerMsg = loggerMsg;
      this.msg = msg;
    }

    public int getCode() {
      return code;
    }

    public String getLoggerMsg() {
      return loggerMsg;
    }

    public String getMsg() {
      return msg;
    }
  }

  public static Path createFile(final String destFolder, final String file) {
    return createFile(destFolder, file, null);
  }

  public static Path createFile(final String destFolder, final String file, final String data) {
    final File dataFolder = new File(destFolder);
    // tries to make directory
    if (Files.notExists(Paths.get(destFolder)) && !dataFolder.mkdirs()) {
      exit(MKDIRS_FAILED, destFolder);
    }

    final Path pathToFile = Paths.get(destFolder, file);

    if(Files.exists(pathToFile)) {
      deleteWhenExists(pathToFile);
    }

    if (data != null) {
      writeToFile(pathToFile, data);
    }

    return pathToFile;
  }

  public static void writeToFile(final Path pathToFile, final String data) {
    writeFile(pathToFile, data, false);
  }

  public static void appendToFile(final Path pathToFile, final String data) {
    writeFile(pathToFile, data, true);
  }

  public static void exit(final ExitStatus exitStatus, final String errorSource) {
    final String detail = errorSource == null ? NO_DETAIL : errorSource;
    LOGGER.error(exitStatus.loggerMsg, detail);
    System.out.println(exitStatus.getMsg());
    System.exit(exitStatus.getCode());
  }

  // private methods

  /**
   * Try to delete a file, knowing that it exists.
   * If the file cannot be deleted, program is aborted with the corresponding exit code
   * @param pathToFile the file path that refers to the file that will be deleted
   */
  private static void deleteWhenExists(final Path pathToFile) {


    try {
      Files.deleteIfExists(pathToFile);
    } catch(IOException e) {
      exit(DELETE_EXISTING_FILE_FAILED, pathToFile.toString());
    }
  }

  private static boolean writeFile(final Path pathToFile, final String data, final boolean append) {
    BufferedWriter writer = null;
    try {
      writer = new BufferedWriter(new FileWriter(pathToFile.toFile(), append));
      writer.write(data);
      return true;
    } catch (IOException e) {
      LOGGER.warn(ERROR_WRITE_FILE_LOGGER, pathToFile, e);
      System.out.println(writeFailMessage(pathToFile));
      return false;
    } finally {
      try {
        // close the writer regardless of what happens...
        if (writer != null) {
          writer.close();
        }
      } catch (Exception ignored) {

      }
    }
  }

  private static String writeFailMessage(final Path pathToFile) {
    return "[FAIL] - An unexpected error occurred while writing the file '" + pathToFile + "'. \n" +
            "Check the logs for more info.\n" +
            "Aborting...";
  }
}
