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

  private static final String CHECK_LOGS = "\nCheck logs for more info.";
  private static final String ABORTING = CHECK_LOGS + "\nAborting...\n";
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
            "[FAIL] - Could not delete an existing file." + ABORTING),
    WRITE_FILE_ERROR(-11,
            "[FAIL] - An unexpected IO Exception occurred while writing the file. Caused by: ",
            "[FAIL] -  An unexpected IO Exception occurred while writing a file." + CHECK_LOGS);

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

  /**
   * Creates the specified {@code file} at the specified destination folder
   * and saves the specified {@code data} on it.
   * <P>
   * If the destination folder does not exists, it tries to create it.
   * <P>
   * If the file exists, it tries to delete it first.
   * <P>
   * If anything fails during these operations, program is aborted with a detail log and display message,
   * and the corresponding exit status code.
   * @param destFolder destination folder of the new file
   * @param file name of the new file
   * @param data data to be saved on the new file
   * @return the path to the just created file
   */
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
      if (!writeToFile(pathToFile, data)) {
        exit(WRITE_FILE_ERROR, null);
      }
    }

    return pathToFile;
  }

  /**
   * Writes - not appends - {@code data} to the specified file
   *
   * @param pathToFile path to the file where data is going to be written
   * @param data data to be written in file
   * @return true if data could be written; false otherwise
   */
  public static boolean writeToFile(final Path pathToFile, final String data) {
    return writeFile(pathToFile, data, false);
  }

  /**
   * Appends - {@code data} to the specified file
   *
   * @param pathToFile path to the file where data is going to be written
   * @param data data to be written in file
   * @return true if data could be appended; false otherwise
   */
  public static boolean appendToFile(final Path pathToFile, final String data) {
    return writeFile(pathToFile, data, true);
  }

  /**
   * Exits program using the exit status information (code, logger message and standard output message).
   * <P>
   * An errorSource object can be passed so as the logger can show what made the program failed.
   * @param exitStatus exit status enum
   * @param errorSource detail error source for being passed to the logger; can be null if no detail is needed
   */
  public static void exit(final ExitStatus exitStatus, final Object errorSource) {
    final Object reason = errorSource == null ? NO_DETAIL : errorSource;
    writeFailMessages(exitStatus, reason);
    System.exit(exitStatus.getCode());
  }

  // private methods

  private static void writeFailMessages(final ExitStatus exitStatus, final Object reason) {
    LOGGER.error(exitStatus.getLoggerMsg(), reason);
    System.out.println(exitStatus.getMsg());
  }

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

  /**
   * Writes {@code data} to the specified file using or not appended mode accordingly to {@code appended}'s value
   *
   * @param pathToFile path to the file where data is going to be written
   * @param data data to be written in file
   * @param append whether data should be appended or not
   * @return true if data could be written; false otherwise
   */
  private static boolean writeFile(final Path pathToFile, final String data, final boolean append) {
    BufferedWriter writer = null;
    try {
      writer = new BufferedWriter(new FileWriter(pathToFile.toFile(), append));
      writer.write(data);
      return true;
    } catch (IOException e) {
      writeFailMessages(WRITE_FILE_ERROR, e);
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
}
