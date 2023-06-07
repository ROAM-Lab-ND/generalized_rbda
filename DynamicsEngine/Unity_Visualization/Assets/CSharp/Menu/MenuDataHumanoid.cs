using System.Collections.Generic;
using System.Collections.ObjectModel;
using UnityEngine;
using System.IO;
using System.ComponentModel;
using System.Reflection;
using System;

public class MenuDataHumanoid : MonoBehaviour
{
    //define as a singleton to enforce only one global instance
    public static MenuDataHumanoid instance = null;

    [SerializeField] public MenuDataHumanoidValues data = new MenuDataHumanoidValues();

    private bool changed;
    private string current_filename = "default_config_humanoid.json";

    void Awake() {
      if(instance == null) { instance = this; }
      else if (instance != this) { Destroy(gameObject); }
    }

    void Start(){
      LoadFromJson();
      setValues();
    }

    public bool IsOpen() { return true; }

    public void SaveToJson(){
      Debug.Log("Saving file to: " + Application.dataPath + "/" + current_filename);
      string save = JsonUtility.ToJson(data);
      System.IO.File.WriteAllText(Application.dataPath + "/" + current_filename, save);
    }

    public void LoadFromJson(){
      if(System.IO.File.Exists(Application.dataPath + "/" + current_filename)){
        StreamReader reader = new StreamReader(Application.dataPath + "/" + current_filename);
        string read = reader.ReadToEnd();
        Debug.Log(read);
        JsonUtility.FromJsonOverwrite(read, data);
        setValues();
        reader.Close();
      } else {
        Debug.Log("file not found");
      }
    }

    public bool get_cheater_mode() {return data.cheater_mode;}
    public Vector3 get_Kd_body() {return data.Kd_body;}


    public bool get_bool_parameter(string parameter) {
        //Debug.Log("getting " + parameter);
        Type myType = Type.GetType("MenuDataHumanoidValues");
        FieldInfo myFieldInfo = myType.GetField(parameter);
        bool bool_value = (bool) myFieldInfo.GetValue(data);
        // Debug.Log("bool value: " + bool_value + " vs data.cheater_mode: " + data.cheater_mode);
        return bool_value;
    }

    // not using this method yet
    public void set_bool_parameter(string parameter, bool parameter_value) {
        Type myType = Type.GetType("MenuDataHumanoidValues");
        FieldInfo myFieldInfo = myType.GetField(parameter);
        // Debug.Log("before: " + myFieldInfo.GetValue(data));
        myFieldInfo.SetValue(data, parameter_value);
        // Debug.Log("after: " + myFieldInfo.GetValue(data));
    }

    public float get_float_parameter(string parameter){
      //Debug.Log("getting " + parameter);
      Type myType = Type.GetType("MenuDataHumanoidValues");
      FieldInfo myFieldInfo = myType.GetField(parameter);
      float float_value = (float) myFieldInfo.GetValue(data);
      // Debug.Log("got float value: " + float_value );
      return float_value;
    }

    public Vector3 get_vector_parameter(string parameter){
      //Debug.Log("getting " + parameter);
      Type myType = Type.GetType("MenuDataHumanoidValues");
      FieldInfo myFieldInfo = myType.GetField(parameter);
      Vector3 vector_value = (Vector3) myFieldInfo.GetValue(data);
      // Debug.Log("vector value: " + vector_value + " vs data.Kd_body: " + data.Kd_body);
      return vector_value;
    }

    //TODO: clarify this work flow for marking changes

    public bool isChanged() { return changed; }
    public void change() { changed = false; }

    //TODO: make one of these for each field, for the UI to latch onto
    //add to docs about adding new fields

    public int get_control_mode() {return data.control_mode;}
    public void set_control_mode(string control_mode){
      data.control_mode = int.Parse(control_mode);
      changed = true;
    }
//      public static void save_control_mode_from_keyboard( GameObject objectRef, int control_mode){
//           // Acting on objectRef which is the reference to the calling object
//           data.control_mode = control_mode;
//      }

    public void set_cheater_mode(bool cheater_mode){
      data.cheater_mode = cheater_mode;
      changed = true;
    }

    public void set_display_heightmap(bool display_heightmap){
      data.display_heightmap = display_heightmap;
      changed = true;
    }

    public void set_display_path_planning(bool display_path_planning){
      data.display_path_planning = display_path_planning;
      changed = true;
    }

    public void set_display_potential_field(bool display_potential_field){
      data.display_potential_field = display_potential_field;
      changed = true;
    }

    public void set_display_traversability(bool display_traversability){
      data.display_traversability = display_traversability;
      changed = true;
    }

    public void set_Kd_body(string Kd_body){
      data.Kd_body = parseVector("Kd_body", Kd_body);
      changed = true;
    }

    public void set_Kd_cam(string Kd_cam){
      data.Kd_cam = parseVector("Kd_cam", Kd_cam);
      changed = true;
    }

    public void set_Kd_clm(string Kd_clm){
      data.Kd_clm = parseVector("Kd_clm", Kd_clm);
      changed = true;
    }

    public void set_Kd_foot(string Kd_foot){
      data.Kd_foot = parseVector("Kd_foot", Kd_foot);
      changed = true;
    }

    public void set_Kd_joint(string Kd_joint){
      data.Kd_joint = parseVector("Kd_joint", Kd_joint);
      changed = true;
    }

    public void set_Kd_ori(string Kd_ori){
      data.Kd_ori = parseVector("Kd_ori", Kd_ori);
      changed = true;
    }

    public void set_Kp_body(string Kp_body){
      data.Kp_body = parseVector("Kp_body", Kp_body);
      changed = true;
    }

    public void set_Kp_cam(string Kp_cam){
      data.Kp_cam = parseVector("Kp_cam", Kp_cam);
      changed = true;
    }

    public void set_Kp_clm(string Kp_clm){
      data.Kp_clm = parseVector("Kp_clm", Kp_clm);
      changed = true;
    }

    public void set_Kp_foot(string Kp_foot){
      data.Kp_foot = parseVector("Kp_foot", Kp_foot);
      changed = true;
    }

    public void set_Kp_joint(string Kp_joint){
      data.Kp_joint = parseVector("Kp_joint", Kp_joint);
      changed = true;
    }

    public void set_Kp_ori(string Kp_ori){
      data.Kp_ori = parseVector("Kp_ori", Kp_ori);
      changed = true;
    }

    public void set_use_wbc(bool use_wbc){
      data.use_wbc = use_wbc;
      changed = true;
    }

    public void set_use_rc(bool use_rc){
      data.use_rc = use_rc;
      changed = true;
    }

    private Vector3 parseVector(string name, string vector){
      if(vector.StartsWith("(") && vector.EndsWith(")")){
        vector = vector.Substring(1, vector.Length-2);
        string[] array = vector.Split(',');
        if(array.Length == 3){
          Vector3 newVector = new Vector3();
          for(int i = 0; i < 3; i++){
            newVector[i] = float.Parse(array[i]);
          }
          return newVector;
        } else {
          Debug.Log("Error parsing: " + name + " vector of values " + vector + " could not split into 3 values");
        }
      } else {
        Debug.Log("Error parsing: " + name + " vector of values " + vector + " formatted without proper parenthesis");
      }
      return new Vector3();
    }

    //set the displayed fields to be in the same state as the current values
    //TODO: note the significant digits used for vectors
    public void setValues(){
      Debug.Log("setting values");
      Debug.Log(isChanged());
      GameObject.Find("filename_input_field").GetComponent<UnityEngine.UI.InputField>().text = current_filename;
      GameObject.Find("control_mode_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.control_mode.ToString();

      Dictionary<string, bool> bool_dictionary = new Dictionary<string, bool>()
      {
         //{"use_wbc", data.use_wbc},
         //{"use_rc", data.use_rc},
         {"cheater_mode", data.cheater_mode},
         {"display_heightmap", data.display_heightmap},
         {"display_path_planning", data.display_path_planning},
         {"display_potential_field", data.display_potential_field},
         {"display_traversability", data.display_traversability}
      };

      foreach (KeyValuePair<string,bool> kvp in bool_dictionary)
      {
         GameObject.Find(kvp.Key + "_toggle").GetComponent<UnityEngine.UI.Toggle>().isOn = kvp.Value;
         //Debug.Log(kvp.Key + " has value now of " + GameObject.Find(kvp.Key + "_toggle").GetComponent<UnityEngine.UI.Toggle>().isOn.ToString());
      }

      Dictionary<string, Vector3> vector_dictionary = new Dictionary<string, Vector3>()
      {
         {"Kd_body", data.Kd_body},
         {"Kd_cam", data.Kd_cam},
         {"Kd_foot", data.Kd_foot},
         {"Kd_clm", data.Kd_clm},
         {"Kd_joint", data.Kd_joint},
         {"Kd_ori", data.Kd_ori}
      };

      foreach (KeyValuePair<string,Vector3> kvp in vector_dictionary)
      {
         GameObject.Find(kvp.Key + "_input_field").GetComponent<UnityEngine.UI.InputField>().text = kvp.Value.ToString("N3");
      }

      Dictionary<string, float> float_dictionary = new Dictionary<string, float>()
      {
         // ex: {"<field name>", data.<field name>}

      };

      foreach (KeyValuePair<string,float> kvp in float_dictionary)
      {
        GameObject.Find(kvp.Key + "_input_field").GetComponent<UnityEngine.UI.InputField>().text = kvp.Value.ToString();
      }

      changed = true;
    }
}

[System.Serializable]
public class MenuDataHumanoidValues {
  public int control_mode;
  public bool cheater_mode;

  public bool use_wbc;
  public bool use_rc;

  public bool display_heightmap;
  public bool display_path_planning;
  public bool display_potential_field;
  public bool display_traversability;

  public Vector3 Kd_body;
  public Vector3 Kd_cam;
  public Vector3 Kd_clm;
  public Vector3 Kd_foot;
  public Vector3 Kd_joint;
  public Vector3 Kd_ori;

  public Vector3 Kp_body;
  public Vector3 Kp_cam;
  public Vector3 Kp_clm;
  public Vector3 Kp_foot;
  public Vector3 Kp_joint;
  public Vector3 Kp_ori;

}
