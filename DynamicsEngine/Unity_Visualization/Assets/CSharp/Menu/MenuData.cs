using System.Collections.Generic;
using System.Collections.ObjectModel;
using UnityEngine;
using System.IO;
using System.ComponentModel;
using System.Reflection;
using System;

public class MenuData : MonoBehaviour
{
    //define as a singleton to enforce only one global instance
    public static MenuData instance = null;

    [SerializeField] private MenuDataValues data = new MenuDataValues();

    private bool changed;
    private string current_filename = "default_config.json";

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

    public bool get_bool_parameter(string parameter) {
        Type myType = Type.GetType("MenuDataValues");
        FieldInfo myFieldInfo = myType.GetField(parameter);
        // Debug.Log("got: " + myFieldInfo.GetValue(data));
        bool bool_value = (bool) myFieldInfo.GetValue(data);
        Debug.Log("bool value: " + bool_value + " vs data.cheater_mode: " + data.cheater_mode);
        return bool_value;
    }

    // not using yet
    public void set_bool_parameter(string parameter, bool parameter_value) {
        Type myType = Type.GetType("MenuDataValues");
        FieldInfo myFieldInfo = myType.GetField(parameter);
        Debug.Log("change to " + parameter_value);
        Debug.Log("before: " + myFieldInfo.GetValue(data));
        myFieldInfo.SetValue(data, parameter_value);
        Debug.Log("after: " + myFieldInfo.GetValue(data));
    }

    public float get_float_parameter(string parameter){
      Type myType = Type.GetType("MenuDataValues");
      FieldInfo myFieldInfo = myType.GetField(parameter);
      float float_value = (float) myFieldInfo.GetValue(data);
      Debug.Log("float value: " + float_value + " vs data.r_prev: " + data.R_prev);
      return float_value;
    }

    public Vector3 get_vector_parameter(string parameter){
      Type myType = Type.GetType("MenuDataValues");
      FieldInfo myFieldInfo = myType.GetField(parameter);
      Vector3 vector_value = (Vector3) myFieldInfo.GetValue(data);
      Debug.Log("vector value: " + vector_value + " vs data.Kd_body: " + data.Kd_body);
      return vector_value;
    }


    /////////////////




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

    public bool get_cheater_mode() {return data.cheater_mode;}
    public void set_cheater_mode(bool cheater_mode){
      data.cheater_mode = cheater_mode;
      changed = true;
    }

    public bool get_display_heightmap() {return data.display_heightmap;}
    public void set_display_heightmap(bool display_heightmap){
      data.display_heightmap = display_heightmap;
      changed = true;
    }

    public bool get_display_path_planning() {return data.display_path_planning;}
    public void set_display_path_planning(bool display_path_planning){
      data.display_path_planning = display_path_planning;
      changed = true;
    }

    public bool get_display_potential_field() {return data.display_potential_field;}
    public void set_display_potential_field(bool display_potential_field){
      data.display_potential_field = display_potential_field;
      changed = true;
    }

    public bool get_display_traversability() {return data.display_traversability;}
    public void set_display_traversability(bool display_traversability){
      data.display_traversability = display_traversability;
      changed = true;
    }

    public Vector3 get_Kd_body() {return data.Kd_body;}
    public void set_Kd_body(string Kd_body){
      data.Kd_body = parseVector("Kd_body", Kd_body);
      changed = true;
    }

    public Vector3 get_Kd_cam() {return data.Kd_cam;}
    public void set_Kd_cam(string Kd_cam){
      data.Kd_cam = parseVector("Kd_cam", Kd_cam);
      changed = true;
    }

    public Vector3 get_Kd_clm() {return data.Kd_clm;}
    public void set_Kd_clm(string Kd_clm){
      data.Kd_clm = parseVector("Kd_clm", Kd_clm);
      changed = true;
    }

    public Vector3 get_Kd_foot() {return data.Kd_foot;}
    public void set_Kd_foot(string Kd_foot){
      data.Kd_foot = parseVector("Kd_foot", Kd_foot);
      changed = true;
    }

    public Vector3 get_Kd_joint() {return data.Kd_joint;}
    public void set_Kd_joint(string Kd_joint){
      data.Kd_joint = parseVector("Kd_joint", Kd_joint);
      changed = true;
    }

    public Vector3 get_Kd_ori() {return data.Kd_ori;}
    public void set_Kd_ori(string Kd_ori){
      data.Kd_ori = parseVector("Kd_ori", Kd_ori);
      changed = true;
    }

    public Vector3 get_Kp_body() {return data.Kp_body;}
    public void set_Kp_body(string Kp_body){
      data.Kp_body = parseVector("Kp_body", Kp_body);
      changed = true;
    }

    public Vector3 get_Kp_cam() {return data.Kp_cam;}
    public void set_Kp_cam(string Kp_cam){
      data.Kp_cam = parseVector("Kp_cam", Kp_cam);
      changed = true;
    }

    public Vector3 get_Kp_clm() {return data.Kp_clm;}
    public void set_Kp_clm(string Kp_clm){
      data.Kp_clm = parseVector("Kp_clm", Kp_clm);
      changed = true;
    }

    public Vector3 get_Kp_foot() {return data.Kp_foot;}
    public void set_Kp_foot(string Kp_foot){
      data.Kp_foot = parseVector("Kp_foot", Kp_foot);
      changed = true;
    }

    public Vector3 get_Kp_joint() {return data.Kp_joint;}
    public void set_Kp_joint(string Kp_joint){
      data.Kp_joint = parseVector("Kp_joint", Kp_joint);
      changed = true;
    }

    public Vector3 get_Kp_ori() {return data.Kp_ori;}
    public void set_Kp_ori(string Kp_ori){
      data.Kp_ori = parseVector("Kp_ori", Kp_ori);
      changed = true;
    }

    public Vector3 get_Q_ang() {return data.Q_ang;}
    public void set_Q_ang(string Q_ang){
      data.Q_ang = parseVector("Q_ang", Q_ang);
      changed = true;
    }

    public Vector3 get_Q_ori() {return data.Q_ori;}
    public void set_Q_ori(string Q_ori){
      data.Q_ori = parseVector("Q_ori", Q_ori);
      changed = true;
    }

    public Vector3 get_Q_pos() {return data.Q_pos;}
    public void set_Q_pos(string Q_pos){
      data.Q_pos = parseVector("Q_pos", Q_pos);
      changed = true;
    }

    public Vector3 get_Q_vel() {return data.Q_vel;}
    public void set_Q_vel(string Q_vel){
      data.Q_vel = parseVector("Q_vel", Q_vel);
      changed = true;
    }

    public Vector3 get_RPC_H_phi0() {return data.RPC_H_phi0;}
    public void set_RPC_H_phi0(string RPC_H_phi0){
      data.RPC_H_phi0 = parseVector("RPC_H_phi0", RPC_H_phi0);
      changed = true;
    }

    public Vector3 get_RPC_H_r_rot() {return data.RPC_H_r_rot;}
    public void set_RPC_H_r_rot(string RPC_H_r_rot){
      data.RPC_H_r_rot = parseVector("RPC_H_r_rot", RPC_H_r_rot);
      changed = true;
    }

    public Vector3 get_RPC_H_r_trans() {return data.RPC_H_r_trans;}
    public void set_RPC_H_r_trans(string RPC_H_r_trans){
      data.RPC_H_r_trans = parseVector("RPC_H_r_trans", RPC_H_r_trans);
      changed = true;
    }

    public Vector3 get_RPC_H_theta0() {return data.RPC_H_theta0;}
    public void set_RPC_H_theta0(string RPC_H_theta0){
      data.RPC_H_theta0 = parseVector("RPC_H_theta0", RPC_H_theta0);
      changed = true;
    }

    public Vector3 get_RPC_Q_dp() {return data.RPC_Q_dp;}
    public void set_RPC_Q_dp(string RPC_Q_dp){
      data.RPC_Q_dp = parseVector("RPC_Q_dp", RPC_Q_dp);
      changed = true;
    }

    public Vector3 get_RPC_Q_dtheta() {return data.RPC_Q_dtheta;}
    public void set_RPC_Q_dtheta(string RPC_Q_dtheta){
      data.RPC_Q_dtheta = parseVector("RPC_Q_dtheta", RPC_Q_dtheta);
      changed = true;
    }

    public Vector3 get_RPC_Q_p() {return data.RPC_Q_p;}
    public void set_RPC_Q_p(string RPC_Q_p){
      data.RPC_Q_p = parseVector("RPC_Q_p", RPC_Q_p);
      changed = true;
    }

    public Vector3 get_RPC_Q_theta() {return data.RPC_Q_theta;}
    public void set_RPC_Q_theta(string RPC_Q_theta){
      data.RPC_Q_theta = parseVector("RPC_Q_theta", RPC_Q_theta);
      changed = true;
    }

    public Vector3 get_RPC_R_f() {return data.RPC_R_f;}
    public void set_RPC_R_f(string RPC_R_f){
      data.RPC_R_f = parseVector("RPC_R_f", RPC_R_f);
      changed = true;
    }

    public Vector3 get_RPC_R_r() {return data.RPC_R_r;}
    public void set_RPC_R_r(string RPC_R_r){
      data.RPC_R_r = parseVector("RPC_R_r", RPC_R_r);
      changed = true;
    }

    public Vector3 get_RPC_filter() {return data.RPC_filter;}
    public void set_RPC_filter(string RPC_filter){
      data.RPC_filter = parseVector("RPC_filter", RPC_filter);
      changed = true;
    }

    public Vector3 get_RPC_gravity() {return data.RPC_gravity;}
    public void set_RPC_gravity(string RPC_gravity){
      data.RPC_gravity = parseVector("RPC_gravity", RPC_gravity);
      changed = true;
    }

    public Vector3 get_RPC_inertia() {return data.RPC_inertia;}
    public void set_RPC_inertia(string RPC_inertia){
      data.RPC_inertia = parseVector("RPC_inertia", RPC_inertia);
      changed = true;
    }

    public float get_RPC_interface_type() {return data.RPC_interface_type;}
    public void set_RPC_interface_type(string RPC_interface_type){
      data.RPC_interface_type = float.Parse(RPC_interface_type);
      changed = true;
    }

    public float get_RPC_mass() {return data.RPC_mass;}
    public void set_RPC_mass(string RPC_mass){
      data.RPC_mass = float.Parse(RPC_mass);
      changed = true;
    }

    public float get_RPC_mu() {return data.RPC_mu;}
    public void set_RPC_mu(string RPC_mu){
      data.RPC_mu = float.Parse(RPC_mu);
      changed = true;
    }

    public float get_RPC_use_async_filt() {return data.RPC_use_async_filt;}
    public void set_RPC_use_async_filt(string RPC_use_async_filt){
      data.RPC_use_async_filt = float.Parse(RPC_use_async_filt);
      changed = true;
    }

    public float get_RPC_use_pred_comp() {return data.RPC_use_pred_comp;}
    public void set_RPC_use_pred_comp(string RPC_use_pred_comp){
      data.RPC_use_pred_comp = float.Parse(RPC_use_pred_comp);
      changed = true;
    }

    public float get_RPC_visualize_pred() {return data.RPC_visualize_pred;}
    public void set_RPC_visualize_pred(string RPC_visualize_pred){
      data.RPC_visualize_pred = float.Parse(RPC_visualize_pred);
      changed = true;
    }

    public float get_R_control() {return data.R_control;}
    public void set_R_control(string R_control){
      data.R_control = float.Parse(R_control);
      changed = true;
    }

    public float get_R_prev() {return data.R_prev;}
    public void set_R_prev(string R_prev){
      data.R_prev = float.Parse(R_prev);
      changed = true;
    }

    public Vector3 get_Swing_Kd_cartesian() {return data.Swing_Kd_cartesian;}
    public void set_Swing_Kd_cartesian(string Swing_Kd_cartesian){
      data.Swing_Kd_cartesian = parseVector("Swing_Kd_cartesian", Swing_Kd_cartesian);
      changed = true;
    }

    public Vector3 get_Swing_Kd_joint() {return data.Swing_Kd_joint;}
    public void set_Swing_Kd_joint(string Swing_Kd_joint){
      data.Swing_Kd_joint = parseVector("Swing_Kd_joint", Swing_Kd_joint);
      changed = true;
    }

    public Vector3 get_Swing_Kp_cartesian() {return data.Swing_Kp_cartesian;}
    public void set_Swing_Kp_cartesian(string Swing_Kp_cartesian){
      data.Swing_Kp_cartesian = parseVector("Swing_Kp_cartesian", Swing_Kp_cartesian);
      changed = true;
    }

    public Vector3 get_Swing_Kp_joint() {return data.Swing_Kp_joint;}
    public void set_Swing_Kp_joint(string Swing_Kp_joint){
      data.Swing_Kp_joint = parseVector("Swing_Kp_joint", Swing_Kp_joint);
      changed = true;
    }

    public Vector3 get_Swing_step_offset() {return data.Swing_step_offset;}
    public void set_Swing_step_offset(string Swing_step_offset){
      data.Swing_step_offset = parseVector("Swing_step_offset", Swing_step_offset);
      changed = true;
    }

    public float get_Swing_traj_height() {return data.Swing_traj_height;}
    public void set_Swing_traj_height(string Swing_traj_height){
      data.Swing_traj_height = float.Parse(Swing_traj_height);
      changed = true;
    }

    public float get_Swing_use_tau_ff() {return data.Swing_use_tau_ff;}
    public void set_Swing_use_tau_ff(string Swing_use_tau_ff){
      data.Swing_use_tau_ff = float.Parse(Swing_use_tau_ff);
      changed = true;
    }

    public float get_acro_task() {return data.acro_task;}
    public void set_acro_task(string acro_task){
      data.acro_task = float.Parse(acro_task);
      changed = true;
    }

    public Vector3 get_des_dp() {return data.des_dp;}
    public void set_des_dp(string des_dp){
      data.des_dp = parseVector("des_dp", des_dp);
      changed = true;
    }

    public Vector3 get_des_dp_max() {return data.des_dp_max;}
    public void set_des_dp_max(string des_dp_max){
      data.des_dp_max = parseVector("des_dp_max", des_dp_max);
      changed = true;
    }

    public Vector3 get_des_dtheta() {return data.des_dtheta;}
    public void set_des_dtheta(string des_dtheta){
      data.des_dtheta = parseVector("des_dtheta", des_dtheta);
      changed = true;
    }

    public Vector3 get_des_dtheta_max() {return data.des_dtheta_max;}
    public void set_des_dtheta_max(string des_dtheta_max){
      data.des_dtheta_max = parseVector("des_dtheta_max", des_dtheta_max);
      changed = true;
    }

    public Vector3 get_des_p() {return data.des_p;}
    public void set_des_p(string des_p){
      data.des_p = parseVector("des_p", des_p);
      changed = true;
    }

    public Vector3 get_des_theta() {return data.des_theta;}
    public void set_des_theta(string des_theta){
      data.des_theta = parseVector("des_theta", des_theta);
      changed = true;
    }

    public Vector3 get_des_theta_max() {return data.des_theta_max;}
    public void set_des_theta_max(string des_theta_max){
      data.des_theta_max = parseVector("des_theta_max", des_theta_max);
      changed = true;
    }

    public Vector3 get_gait_disturbance() {return data.gait_disturbance;}
    public void set_gait_disturbance(string gait_disturbance){
      data.gait_disturbance = parseVector("gait_disturbance", gait_disturbance);
      changed = true;
    }

    public float get_gait_max_leg_angle() {return data.gait_max_leg_angle;}
    public void set_gait_max_leg_angle(string gait_max_leg_angle){
      data.gait_max_leg_angle = float.Parse(gait_max_leg_angle);
      changed = true;
    }

    public float get_gait_max_stance_time() {return data.gait_max_stance_time;}
    public void set_gait_max_stance_time(string gait_max_stance_time){
      data.gait_max_stance_time = float.Parse(gait_max_stance_time);
      changed = true;
    }

    public float get_gait_min_stance_time() {return data.gait_min_stance_time;}
    public void set_gait_min_stance_time(string gait_min_stance_time){
      data.gait_min_stance_time = float.Parse(gait_min_stance_time);
      changed = true;
    }

    public float get_gait_override() {return data.gait_override;}
    public void set_gait_override(string gait_override){
      data.gait_override = float.Parse(gait_override);
      changed = true;
    }

    public float get_gait_period_time() {return data.gait_period_time;}
    public void set_gait_period_time(string gait_period_time){
      data.gait_period_time = float.Parse(gait_period_time);
      changed = true;
    }

    public Vector3 get_gait_recovery() {return data.gait_recovery;}
    public void set_gait_recovery(string gait_recovery){
      data.gait_recovery = parseVector("gait_recovery", gait_recovery);
      changed = true;
    }

    public float get_gait_switching_phase() {return data.gait_switching_phase;}
    public void set_gait_switching_phase(string gait_switching_phase){
      data.gait_switching_phase = float.Parse(gait_switching_phase);
      changed = true;
    }

    public float get_gait_type() {return data.gait_type;}
    public void set_gait_type(string gait_type){
      data.gait_type = float.Parse(gait_type);
      changed = true;
    }

    public float get_stance_legs() {return data.stance_legs;}
    public void set_stance_legs(string stance_legs){
      data.stance_legs = float.Parse(stance_legs);
      changed = true;
    }

    public bool get_use_wbc() {return data.use_wbc;}
    public void set_use_wbc(bool use_wbc){
      data.use_wbc = use_wbc;
      changed = true;
    }

    public bool get_use_rc() {return data.use_rc;}
    public void set_use_rc(bool use_rc){
      data.use_rc = use_rc;
      changed = true;
    }

    /*public Vector3 get_replacem4() {return data.replacem4;}
    public void set_replacem4(string replacem4){
      data.replacem4 = parseVector("replacem4", replacem4);
      changed = true;
    }

    public float get_replacem3() {return data.replacem3;}
    public void set_replacem3(string replacem3){
      data.replacem3 = replacem3;
      changed = true;
    }*/

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

      //TODO: fix finding these on inactive pages

      Dictionary<string, Vector3> vector_dictionary = new Dictionary<string, Vector3>()
      {
         {"Kd_body", data.Kd_body},
         {"Kd_cam", data.Kd_cam},
         {"Kd_foot", data.Kd_foot},
         {"Kd_clm", data.Kd_clm},
         {"Kd_joint", data.Kd_joint},
         {"Kd_ori", data.Kd_ori},

         {"Q_ang", data.Q_ang},
         {"Q_ori", data.Q_ori},
         {"Q_pos", data.Q_pos},
         {"Q_vel", data.Q_vel},

         {"RPC_H_phi0", data.RPC_H_phi0},
         {"RPC_H_r_rot", data.RPC_H_r_rot},
         {"RPC_H_r_trans", data.RPC_H_r_trans},
         {"RPC_H_theta0", data.RPC_H_theta0},

         {"RPC_Q_dp", data.RPC_Q_dp},
         {"RPC_Q_dtheta", data.RPC_Q_dtheta},
         {"RPC_Q_p", data.RPC_Q_p},
         {"RPC_Q_theta", data.RPC_Q_theta},

         {"RPC_R_f", data.RPC_R_f},
         {"RPC_R_r", data.RPC_R_r},

         {"RPC_filter", data.RPC_filter},
         {"RPC_gravity", data.RPC_gravity},
         {"RPC_inertia", data.RPC_inertia},

         {"Swing_Kd_cartesian", data.Swing_Kd_cartesian},
         {"Swing_Kp_cartesian", data.Swing_Kp_cartesian},
         {"Swing_Kp_joint", data.Swing_Kp_joint},
         {"Swing_step_offset", data.Swing_step_offset},

         {"des_dp", data.des_dp},
         {"des_dp_max", data.des_dp_max},
         {"des_dtheta", data.des_dtheta},
         {"des_dtheta_max", data.des_dtheta_max},
         {"des_p", data.des_p},
         {"des_theta", data.des_theta},
         {"des_theta_max", data.des_theta_max},

         {"gait_disturbance", data.gait_disturbance},
         {"gait_recovery", data.gait_recovery},

      };

      foreach (KeyValuePair<string,Vector3> kvp in vector_dictionary)
      {
         GameObject.Find(kvp.Key + "_input_field").GetComponent<UnityEngine.UI.InputField>().text = kvp.Value.ToString("N3");
      }

      Dictionary<string, float> float_dictionary = new Dictionary<string, float>()
      {
         {"RPC_interface_type", data.RPC_interface_type},
         {"RPC_mass", data.RPC_mass},
         {"RPC_mu", data.RPC_mu},
         {"RPC_use_async_filt", data.RPC_use_async_filt},

         // {"RPC_interface_type", data.RPC_interface_type},
         // {"RPC_mass", data.RPC_mass},
         // {"RPC_mu", data.RPC_mu},
         // {"RPC_use_async_filt", data.RPC_use_async_filt},
         {"RPC_use_pred_comp", data.RPC_use_pred_comp},
         {"RPC_visualize_pred", data.RPC_visualize_pred},
         {"R_control", data.R_control},
         {"R_prev", data.R_prev},

         {"Swing_traj_height", data.Swing_traj_height},
         {"Swing_use_tau_ff", data.Swing_use_tau_ff},
         {"acro_task", data.acro_task},

         {"gait_max_leg_angle", data.gait_max_leg_angle},
         {"gait_max_stance_time", data.gait_max_stance_time},
         {"gait_min_stance_time", data.gait_min_stance_time},
         {"gait_override", data.gait_override},
         {"gait_period_time", data.gait_period_time},
         {"gait_switching_phase", data.gait_switching_phase},
         {"gait_type", data.gait_type},

         {"stance_legs", data.stance_legs}

      };

      foreach (KeyValuePair<string,float> kvp in float_dictionary)
      {
        GameObject.Find(kvp.Key + "_input_field").GetComponent<UnityEngine.UI.InputField>().text = kvp.Value.ToString();
      }

      // GameObject.Find("Q_ang_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.Q_ang.ToString("N3");
      // GameObject.Find("Q_ori_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.Q_ori.ToString("N3");
      // GameObject.Find("Q_pos_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.Q_pos.ToString("N3");
      // GameObject.Find("Q_vel_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.Q_vel.ToString("N3");

      // GameObject.Find("RPC_H_phi0_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.RPC_H_phi0.ToString("N3");
      // GameObject.Find("RPC_H_r_rot_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.RPC_H_r_rot.ToString("N3");
      // GameObject.Find("RPC_H_r_trans_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.RPC_H_r_trans.ToString("N3");
      // GameObject.Find("RPC_H_theta0_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.RPC_H_theta0.ToString("N3");

      // GameObject.Find("RPC_Q_dp_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.RPC_Q_dp.ToString("N3");
      // GameObject.Find("RPC_Q_dtheta_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.RPC_Q_dtheta.ToString("N3");
      // GameObject.Find("RPC_Q_p_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.RPC_Q_p.ToString("N3");
      // GameObject.Find("RPC_Q_theta_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.RPC_Q_theta.ToString("N3");

      // GameObject.Find("RPC_R_f_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.RPC_R_f.ToString("N3");
      // GameObject.Find("RPC_R_r_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.RPC_R_r.ToString("N3");

      // GameObject.Find("RPC_filter_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.RPC_filter.ToString("N3");
      // GameObject.Find("RPC_gravity_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.RPC_gravity.ToString("N3");
      // GameObject.Find("RPC_inertia_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.RPC_inertia.ToString("N3");

      // //GameObject.Find("RPC_interface_type_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.RPC_interface_type.ToString();
      // //GameObject.Find("RPC_mass_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.RPC_mass.ToString();
      // //GameObject.Find("RPC_mu_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.RPC_mu.ToString();
      // //GameObject.Find("RPC_use_async_filt_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.RPC_use_async_filt.ToString();
      // GameObject.Find("RPC_use_pred_comp_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.RPC_use_pred_comp.ToString();
      // GameObject.Find("RPC_visualize_pred_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.RPC_visualize_pred.ToString();
      // GameObject.Find("R_control_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.R_control.ToString();
      // GameObject.Find("R_prev_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.R_prev.ToString();

      // GameObject.Find("Swing_Kd_cartesian_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.Swing_Kd_cartesian.ToString("N3");
      // GameObject.Find("Swing_Kd_joint_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.Swing_Kd_joint.ToString("N3");
      // GameObject.Find("Swing_Kp_cartesian_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.Swing_Kp_cartesian.ToString("N3");
      // GameObject.Find("Swing_Kp_joint_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.Swing_Kp_joint.ToString("N3");
      // GameObject.Find("Swing_step_offset_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.Swing_step_offset.ToString("N3");
      // GameObject.Find("Swing_traj_height_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.Swing_traj_height.ToString();
      // GameObject.Find("Swing_use_tau_ff_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.Swing_use_tau_ff.ToString();
      // GameObject.Find("acro_task_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.acro_task.ToString();
      //
      // GameObject.Find("des_dp_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.des_dp.ToString("N3");
      // GameObject.Find("des_dp_max_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.des_dp_max.ToString("N3");
      // GameObject.Find("des_dtheta_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.des_dtheta.ToString("N3");
      // GameObject.Find("des_dtheta_max_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.des_dtheta_max.ToString("N3");
      // GameObject.Find("des_p_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.des_p.ToString("N3");
      // GameObject.Find("des_theta_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.des_theta.ToString("N3");
      // GameObject.Find("des_theta_max_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.des_theta_max.ToString("N3");
      //
      // GameObject.Find("gait_disturbance_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.gait_disturbance.ToString("N3");
      // GameObject.Find("gait_max_leg_angle_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.gait_max_leg_angle.ToString();
      // GameObject.Find("gait_max_stance_time_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.gait_max_stance_time.ToString();
      // GameObject.Find("gait_min_stance_time_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.gait_min_stance_time.ToString();
      // GameObject.Find("gait_override_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.gait_override.ToString();
      // GameObject.Find("gait_period_time_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.gait_period_time.ToString();
      // GameObject.Find("gait_recovery_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.gait_recovery.ToString("N3");
      // GameObject.Find("gait_switching_phase_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.gait_switching_phase.ToString();
      // GameObject.Find("gait_type_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.gait_type.ToString();

      // GameObject.Find("stance_legs_input_field").GetComponent<UnityEngine.UI.InputField>().text = data.stance_legs.ToString();

      changed = true;
    }
}

[System.Serializable]
public class MenuDataValues {
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

  public Vector3 Q_ang;
  public Vector3 Q_ori;
  public Vector3 Q_pos;
  public Vector3 Q_vel;

  public Vector3 RPC_H_phi0;
  public Vector3 RPC_H_r_rot;
  public Vector3 RPC_H_r_trans;
  public Vector3 RPC_H_theta0;
  public Vector3 RPC_Q_dp;
  public Vector3 RPC_Q_dtheta;
  public Vector3 RPC_Q_p;
  public Vector3 RPC_Q_theta;
  public Vector3 RPC_R_f;
  public Vector3 RPC_R_r;
  public Vector3 RPC_filter;
  public Vector3 RPC_gravity;
  public Vector3 RPC_inertia;

  public float RPC_interface_type;
  public float RPC_mass;
  public float RPC_mu;
  public float RPC_use_async_filt;
  public float RPC_use_pred_comp;
  public float RPC_visualize_pred;
  public float R_control;
  public float R_prev;

  public Vector3 Swing_Kd_cartesian;
  public Vector3 Swing_Kd_joint;
  public Vector3 Swing_Kp_cartesian;
  public Vector3 Swing_Kp_joint;
  public Vector3 Swing_step_offset;

  public float Swing_traj_height;
  public float Swing_use_tau_ff;
  public float acro_task;

  public Vector3 des_dp;
  public Vector3 des_dp_max;
  public Vector3 des_dtheta;
  public Vector3 des_dtheta_max;
  public Vector3 des_p;
  public Vector3 des_theta;
  public Vector3 des_theta_max;

  public Vector3 gait_disturbance;
  public float gait_max_leg_angle;
  public float gait_max_stance_time;
  public float gait_min_stance_time;
  public float gait_override;
  public float gait_period_time;
  public Vector3 gait_recovery;
  public float gait_switching_phase;
  public float gait_type;

  public float stance_legs;



}
