# Set the right extrusion

Older devices manufactured before November 2023 require absolute extrusion.
For example:
If you have

- a model after November 2023 you have to **check** the option.
- a model before November 2023 you have to **uncheck** the option.

## Background

Older devices may have hardware limitations that affect extrusion behavior. It's important to understand these considerations to ensure optimal performance.

## Extrusion Method

Newer devices support relative extrusion, which offers more flexibility. However, older devices may require absolute extrusion to function correctly.

## Action for Older Devices

Users with older devices should consider adjusting the `use_relative_e_distance` setting in the printer's preferences. Disabling this setting may improve compatibility and performance.

## How to Adjust `use_relative_e_distance`

1. Navigate to the printer settings. <br>
![image](https://github.com/CR-3D/SliCR-3D-V2/assets/78646047/23ae3927-98c2-4314-b4de-13bee1992409)

2. Set the mode to 'Expert' and make sure you are in the 'General' section. <br>
![image](https://github.com/CR-3D/SliCR-3D-V2/assets/78646047/81ddc198-e5b8-4cef-9985-721a1b333bb2)

3. Scroll down to the bottom ('Advanced'), the first option is 'Use relative E distances'(`use_relative_e_distance`). <br>
![image](https://github.com/CR-3D/SliCR-3D-V2/assets/78646047/33d3191d-61f0-44bf-90aa-6c92ec29c513)

4. Toggle the setting to enable or disable it based on your device age. (Devices before November 2023 have to uncheck it.)
5. Save the changes to the printer model.
