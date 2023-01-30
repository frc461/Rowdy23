package frc.robot.Utilities;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public class ShuffleboardTable {
    
    private final ShuffleboardTab tab;
    private final Map<String, SimpleWidget> keyEntryMap = new HashMap<>();
    
    public ShuffleboardTable(String tabName){
        this.tab = Shuffleboard.getTab(tabName);
    }

    private GenericEntry getEntry(String key){
        return keyEntryMap.get(key).getEntry();
    }

    // private NetworkTableEntry getEntry(String key){
    //     return keyEntryMap.get(key).getEntry();
    // }

    private void putEntry(String key, Object value){
        if(!hasKey(key)){
            keyEntryMap.put(key, tab.add(key, value));      
        }else{
            getEntry(key).setValue(value);
        }
    }

    public boolean hasKey(String key){
        return keyEntryMap.containsKey(key);
    }

    public void putNumber(String key, double value){
        putEntry(key, value);
    }

    public double getNumber(String key, double defaultValue){
        if(!this.hasKey(key)){
            putNumber(key, defaultValue);
            return defaultValue;
        }
        return getEntry(key).getDouble(defaultValue);
    }

    public double getNumber(String key){
        return getNumber(key, 0.0);
    }

    public void putBoolean(String key, boolean value){
        putEntry(key, value);
    }

    public boolean getBoolean(String key, boolean defaultValue){
        if(!this.hasKey(key)){
            putBoolean(key, defaultValue);
            return defaultValue;
        }
        return getEntry(key).getBoolean(defaultValue);
    }

    public boolean getBoolean(String key){
        return getBoolean(key, false);
    }

    public void putString(String key, String value){
        putEntry(key, value);
    }

    public String getString(String key, String defaultValue){
        if(!this.hasKey(key)){
            putString(key, defaultValue);
            return defaultValue;
        }
        return getEntry(key).getString(defaultValue);
    }

    public String getString(String key){
        return getString(key, "");
    }

    
    public static ShuffleboardTable fromJSON(JSONObject json){
        ShuffleboardTable table = new ShuffleboardTable((String) json.get("name"));
        JSONArray tabs = (JSONArray) json.get("tabs");

        for(int i = 0; i < tabs.size(); i++){
            JSONObject tab = (JSONObject) tabs.get(i);
            String name = (String) tab.get("name");
            Object startValue = tab.get("startValue");
            table.putEntry(name, startValue);

            int x = Math.toIntExact((long) tab.get("x"));
            int y = Math.toIntExact((long) tab.get("y"));

            table.keyEntryMap.get(name).withPosition(x, y);
        }
        
        return table;
    }

    public static ShuffleboardTable fromJSON(String url) throws IOException, ParseException{
        try {
            BufferedReader reader = new BufferedReader(new FileReader(new File(Filesystem.getDeployDirectory() + "/tabs/" + url)));

            StringBuilder builder = new StringBuilder();
            String line;
            while((line = reader.readLine()) != null){
                builder.append(line);
            }
            reader.close();

            return ShuffleboardTable.fromJSON((JSONObject) new JSONParser().parse(builder.toString()));
        } catch (FileNotFoundException e) {
            e.printStackTrace();
            return new ShuffleboardTable("undef");
        }
    }
}
