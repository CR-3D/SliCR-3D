
# How to create a vendor profile

The vendor profiles allow to easily creates many different profiles from a common base, via an inheritance graph. It's all contained in a single ini file.


## ini format
As the file in in ini format, each section is in the format:
```
[section_id:section_value]
setting_name1 = setting_value1
setting_name2 = setting_value2
```
each setting in a section need to be unique.

## Sections
Here are the different sections of the vendor profile ini file. Each are explained in more detail afterwards
* `[vendor]`: the first section, containing information about the vendor, to be displayed in the wizard, and to help with (auto-)updates.
* `[printer_model]`: allow to group printer Presets into a common section, with its thumbnail for the wizard. it's the entry point to install vendor presets. When installed, the slicer will remember each {vendor-printer_model-variant} key is installed.
* `[printer]`: printer preset with its extruder(s)
* `[filament]`: fff filament preset
* `[print]`: fff print preset
* `[sla_material]`: sla resin preset
* `[sla_print]`: sla print preset

For the printer, filament and print section, there is two kinds of name:
1) `[print:my_print_preset_key]`: this is a preset that is useable in the slicer, it can be selected in the combo box in the interface.
2) `[print:*0.2mmheight_print*]`: this is a preset that isn't visible in the slicer, it's only used as a library of settings to be applied at the end to the first kind

The printer, filament and print section can inherit another one (of the same type) via the `inherit` setting. It can takes a single value or an array, separated by `;`.  Example:
```
[print:*common_print*]
perimeters = 3
...

[print:*one_per_top*]
only_one_perimeter_top = 1

[print:final_print_preset]
inherit = *common_print*; *one_per_top*
fill_density = 15%
```
## `[vendor]`Section 
This section doesn't have any section value, it's just `[vendor]`
The settings are:
* `id`:  the string id of the vendor file.
* `technologies` = The printer technologies available in this files. Can be `FFF`, `SLA` or `FFF;SLA` if this file has both.
* `name`: the displayed string in the wizard. If `{technology}` is present inside, it will be replaced by the technology string (FFF or SLA).
* `full_name`: a displayed string in the wizard. can have `{technology}`.
* `config_version`: the current version to see if this file is newer , the same or older than the one installed in the slicer. See below how it works.
* `config_update_url`: the url to call to check for new versions. Left it empty if you don't have it.
* `changelog_url`: the url for the changelog of this file. Can be left empty.

### version
The version has X.X.X.X+metadata-prerelease
Example of valid versions:
`2.7`
`2.7.0`
`2.7.61.12345`
`2.7.1.1-alpha2`
`2.7.1.1-beta1`
`2.7.1.1+2024.01.23`
`2.7.1.1+2024.01.23-susi`
if the version is 'bigger', then it's more recent can replace an older version.
How to check if one version is bigger/ more recent than the other?
First we check the first number: `2.0` > `1.10`
If equal, then we check the second number: `2.10` > `2.9`
If equal, then we check the third number: `2.7.1234` > `2.7.0`
If equal, then we check the third number: `2.7.0.1` > `2.7.0.0`
If equal, then we check the prerelease string (alphanumeric order): `2.7.0.1-b1` > `2.7.0.1-a2`, or if it exists `2.7.0.1` > `2.7.0.1-b`

note: the metadata part is ignored
note: if the number of digit is different, then the comparison only look at the first ones`2.7.1` == `2.7`, `2.7` > `2.7.1-alpha`

## `[printer_model]`Section 
The section has a value. It's a id to link the printer_model with printer profiles. Example: `[printer_model:Voron_v4_250]`
The settings are:
* `name`: Displayed name in the wizard for this printer model.
* `variants`: array of string separated by ';' to link a printer preset to a variant. These strings are displayed in the wizard in this order, with the first being the 'default' for this printer model. Example: `0.4; 0.25; highflow 0.6; 0.4 with 5 filaments`
* `technology`: FFF or SLA
`family`: all printer_model with the same family string are placed on the same line. these families are in the same order as in this file.
* `bed_model`: name of the stl file in your vendor directory that contains the bed geometry (example: `bed_model = bed-v4-250.stl`)
* `bed_texture`: name of the png file in your vendor directory that contains the bed texture (example: `bed_texture= bed-v4-250.png`)
* `bed_with_grid`: 0 or 1 if you want to have a grid displayed on top of the bed texture.
* `thumbnail`: name of the png file in your vendor directory that contains the thumbnail to display in the wizard (example: `thumbnail = Voron_v4_250_thumb.png`)
* `default_materials`: array of filaments/materials name to install by default when installing at least one of this printer model. example: `default_materials = Basic ABS @VORON; Basic PLA @VORON;`

## print/sla_print/printer/filament/sla_material

The section value is used as an id to reference the printers preset for the inheritance graph. There are two kind of presets: the normal ones that can be displayed and used and the hidden ones that have a `*` as the first and last character and are only used as intermediate preset in the inheritance tree.
Examples: `[printer:*common*]` `[printer:Printer project 42-id.2]` `[printer:my_printer_id]`


### Slicer settings
To define a setting in a print, filament or printer preset, you put its key (look at the tooltip in the slicer), then the value after an  `=``.
* Boolean type: put `0` for false and `1` for true.
* Integer type: write the number
* Float type: write it with a `.` as zero-separator (`42.1234`)
* Percentage: like a Float with a `%` after (`42.1234%`)
* String type: you can write it directly after the `=`  (`start_gcode = #custom start\nprint_start EXTRUDER=200`). Some`\` will be interpreted: `\n` for new line, `\\` for a '\', `\n` for 'carriage return' ). You can also enclosed it into `"`: (`start_gcode = "#custom start\nprint_start EXTRUDER=200"`)
* Enum type: you need to get the value key for the option you need. The easiest is to get it from a saved config (from the end of a gcode or a saved profile). Example: `fill_pattern = gyroid`
* Point type: it's FloatxFloat (`thumbnails = 64x64`)
* Graph type: it's a vector of value separated by `:`. the first three value are: begin_index, end_index, curve_type(0=SQUARE, 1=LINEAR, 2=SPLINE) and all the other values are the points of the graph. Example: `overhangs_dynamic_speed= 0:3:1:0x0:50x25:100x100`

If the settings is an array, the values need to be separated by a `,`: `thumbnails = 64x64,400x300`
If the setting is deactivated, put a `!` before: `first_layer_extrusion_width = !0.42`
For an array, you can deactivate each value the same way: `idle_temperature = !30,!30`

It's not possible to replace only one element of a vector, you need to set the full vector.

### inheritance

The `inherits` setting allow to start this preset with some settings already set. it's an array using `;` as separator. It's not mandatory.
Examples: `inherits = *common*; *hot*` `inherits = my_printer_id; *slow*` 

The preset start with nothing defined.
The first preset in `inherits` can set the values it defines.
Same for the next ones in the `inherits` array, and it erase the current value if it's already defined.
Then the settings defined in this Preset ca erase anything already defined by the `inherits`.

Advice about inheritance:
* If you define it, put it first after the section name, to increase its visibility.
* There is two kind of presets: 
  1) The first kind defines every settings, by themselves or from an inherited preset. We call them 'full preset'
  2) The second kind only defines a subset of the settings, and so don't inherit any 'full preset'. We call them 'flavour preset'
* Any final preset (without `*` as first and last character in their id) need to be a 'full preset'.
* Hidden presets (with `*` as first and last character in their id) can be 'full preset' or 'flavour preset'.
* A final preset can inherit a final preset.
* A 'full preset' should defines all settings or inherit another 'full preset'. Any setting that isn't defined in a final Preset won't be able to be reverted to 'system value' in the slicer.
* If a 'full preset' inherit another 'full preset', it needs to be the first in the `inherits` array (anything before will be erased by it).
* It's advised to use the word 'common' in any hidden 'full preset', to be sure you only put them as the first inheritance. It's easy to have a `*common*` root that defines everything and then specialised 'commons' like `*common_voron2*` that inherit another common. And of course avoid naming any 'flavour preset' as 'common'.
## `[printer]` Section
The printer section has some special setting:
* `printer_model`: Any final Preset (without `*` as first and last character in their id) need to have this setting defined (by itself or by inheritance) to link it to a printer_model. The value of this setting need to be the same as the printer_model section value. Example: `printer_model = Voron_v4_250`
* `printer_variant`: Any final Preset (without `*` as first and last character in their id) need to have this setting defined (by itself or by inheritance) to link it to a printer_model variant. The value of this setting need to be the same as one of the `variants` values of the linked printer_model. Examples: `printer_variant = 0.4` `printer_variant = highflow 0.6`
* `default_print_profile`: the print section value that should be the default when you select this printer.
* `default_filament_profile`: the filament section value that should be the default when you select this printer.
* `default_sla_print_profile`: the print_sla section value that should be the default when you select this printer.
* `default_sla_material_profile`: the sla_material section value that should be the default when you select this printer.

## `[print]`  and `[sla_print]` Section
it's advised to set a `compatible_printers_condition` or a `compatible_printers` to let the user select this print option only when the desired printer(s) is selected. It avoids polluting the interface's print profile list with print profile from other vendors.

## `[filament]`  and `[sla_material]` Section
You may want to set `compatible_printers_condition` or a `compatible_printers` for the default material profile (like "basic PLA") to not pollute the material list with a "basic pla" for each vendor installed.
If you define a special material only defined in this vendor profile, you can let these field blank to allow any printer to use this material.

You can also use `compatible_prints` and `compatible_prints_condition` if you need to have special filament setting for a special kind of print profile (like a speedy print profile that need higher temperature & more cooling).

## Tool
If you are able to build the slicer, you can build the project `convert_config`.
This tool allow to check & fix a vendor profile.
If you launch it with the path of your profile, it will tell you the issues the profiles has (if you feed it with a profile from prusaslicer, it will tell you all the conversions it makes). Then it will create a fixed config in a new directory (named 'convert'), adding any missing setting into the common roots.
Take care of any "error" or "warning" line, these indicates problems you need to take care manually (like a preset inheriting two presets that both defines everything).
If you want to submit a new vendor or a revision of a current profile for a certain version of the slicer, then the ini file should be unchanged when going through this tool.
