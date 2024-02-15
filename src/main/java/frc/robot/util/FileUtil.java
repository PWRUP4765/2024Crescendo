package frc.robot.util;

import com.google.gson.Gson;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStreamReader;

public class FileUtil {

  /**
   * @param file the string path to file from which to read
   * @return the String format of the file
   */
  public static String getfileString(String file) {
    StringBuilder builder = new StringBuilder();

    try (BufferedReader buffer = new BufferedReader(new FileReader(file))) {
      String str;

      while ((str = buffer.readLine()) != null) {
        builder.append(str).append("\n");
      }
    } catch (IOException e) {
      e.printStackTrace();
    }

    return builder.toString();
  }

  /**
   * @param contents the string contents of the file
   * @return returns the JsonElement which can be used to read stuff from
   */
  public static JsonElement getJsonElement(String contents) {
    return JsonParser.parseString(contents);
  }

  /**
   * @param file the String file
   * @return returns the JsonElement which can be used to read stuff from
   */
  public static JsonObject getJsonObjectFile(String file) {
    return getJsonElement(getfileString(file)).getAsJsonObject();
  }
}
