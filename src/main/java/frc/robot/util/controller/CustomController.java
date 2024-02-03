package frc.robot.util.controller;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import frc.robot.error.NoChannelFoundException;
import frc.robot.error.NoConfigFoundException;
import frc.robot.util.FileUtil;
import java.util.ArrayList;
import java.util.List;

/**
 * @note not done yet
 * @author godbrigero
 */
public final class CustomController extends LogitechController {

  final String FILE_PATH = "ControllerUserConfig.json";

  public CustomController(int port, String configName)
    throws NoChannelFoundException, NoConfigFoundException {
    super(port);
    JsonObject json = FileUtil.getJsonObjectFile(FILE_PATH);

    if (!json.has(configName)) throw new NoConfigFoundException();

    JsonArray arr = json.get(configName).getAsJsonArray();
    List<Integer> newArr = new ArrayList<>();
    for (int i = 0; i < arr.size(); i++) {
      newArr.add(arr.get(i).getAsInt());
    }

    setValues(newArr);
  }
}
