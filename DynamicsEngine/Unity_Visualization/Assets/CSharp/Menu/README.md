# How to add menu parameters:
If parameter is not already in menu follow these steps.

## Add parameter to lcm msg type [robot]_menu_data_lcmt.lcm

## Create a button in unity:
- There are two types of inputs (toggles and input fields)
- Copy and paste a button game object from the desired type and change the game object and children’s name to match the new parameter name
  - In the unity hierarchy you can find a button game object under Environment/Menu/Content/Pages/PageX/Viewport/Content/[parameter]
- Move the position of the game object so it doesn’t overlap with original button
- Click on the [parameter]_input_field or [parameter]_toggle to see the inspector.
  - Scroll down to find On End Edit (for input field) or On Value Changed (boolean).
  - Update the function MenuDataHumanoid.[parameter] to be the correct parameter under the dynamic section
[note: parameter may be in unity menu but if not in humanoid_menu_data.lcmt.lcm then it is not active

## Add parameter to menudata[humanoid].cs
- Define parameter under MenuData[robot]Values (at bottom of file)
- Create set_[parameter] method for new parameter (no get is needed)
- Set game object parameter text in the setValues() method:
  - For floats, vectors, bools:
    - Add key value pair into the corresponding parameter type dictionary: {“[parameter name]”, data.[parameter name]}
  - Other:
    - Follow format of GameObject.Find(“[control_mode or filename]...” …

## Add parameter to default_config_[robot].json
in simulator/unity_Visualization/Assets

## Add the parameter to be sent over LCM in [robot]_parameter_publish.cs:
- Assign menuMsg.[parameter] to the MenuData[robot] unity instance inside the if(MenuData.instance.isChanged()){} statement
- Use get function get_[parameter type]_parameter([string of parameter name]); where parameter type can be bool, float, or vector
- See examples for humanoid:
  - Single bool parameter→
    ```
    menuMsg.cheater_mode = MenuDataHumanoid.instance.get_bool_parameter(“cheater_mode”);
    ```
  - Array of size 3 →
    ```
    convertVector3toArray(menuMsg.Kd_body,MenuData.instance.get_vector_parameter(“Kd_body”));
    ```
  - float →
    ```
    menuMsg.R_prev = MenuData.instance.get_float_parameter(“R_prev”));
    ```

Based on Daniel’s doc: https://docs.google.com/document/d/1_w2P134NRidsNmfScgWCCDle0BinPuQLo88l6ZID9ck/edit  
