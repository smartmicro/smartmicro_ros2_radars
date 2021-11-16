{
   "format_version_major" : 1,
   "format_version_minor" : 0,
   "format_version_patch" : 0,
   "name"                 : "auto_interface_command",
   "section"              : 1000,
   "comment"              : "Maintain compatible section 1000 commands",
   "commands"             : [
      {
         "id"            : 1,
         "name"          : "comp_sensor_reset_delayed_app_start",
         "comment"       : "Perform a device reset and stay the given value [seconds] in the bootloader at next startup (3074.2)",
         "public"        : true,
         "level"         : 0,
         "argument"      : "delay_time",
         "unit"          : "s"
      },
      {
         "id"            : 303,
         "name"          : "comp_fsm_core0_opmode",
         "comment"       : "Select top level FSM operation mode (3078.1)",
         "public"        : true,
         "level"         : 0,
         "argument"      : "opmode"
      },
      {
         "id"            : 340,
         "name"          : "comp_eeprom_ctrl_factory_reset",
         "comment"       : "Performs factory reset (3075.4)",
         "public"        : true,
         "level"         : 0,
         "argument"      : "cmd_value"
      },
      {
         "id"            : 342,
         "name"          : "comp_sensor_reset",
         "comment"       : "Reset command which starts from BIOS (if available) or bootloader (3074.1)",
         "public"        : true,
         "level"         : 0,
         "argument"      : "dummy"
      },
      {
         "id"            : 343,
         "name"          : "comp_pdi_requestor_can",
         "comment"       : "Send PDI data to client (3076.1)",
         "public"        : true,
         "level"         : 0,
         "argument"      : "client_id"
      },
      {
         "id"            : 344,
         "name"          : "comp_eeprom_ctrl_save_param_sec",
         "comment"       : "Save the parameter inside the EEPROM. (3075.3)",
         "public"        : true,
         "level"         : 0,
         "argument"      : "section"
      },
      {
         "id"            : 345,
         "name"          : "comp_eeprom_ctrl_reset_param_sec",
         "comment"       : "Restore default values in RAM. EEPROM content is not changed. (3075.2)",
         "public"        : true,
         "level"         : 0,
         "argument"      : "section"
      },
      {
         "id"            : 346,
         "name"          : "comp_eeprom_ctrl_default_param_sec",
         "comment"       : "Restore default values in RAM and EEPROM. (3075.1)",
         "public"        : true,
         "level"         : 0,
         "argument"      : "section"
      },
      {
         "id"            : 350,
         "name"          : "comp_timebase_set_seconds_val",
         "comment"       : "Set SECONDS value of NTP UTC timestamp",
         "public"        : true,
         "level"         : 0,
         "argument"      : "seconds",
         "unit"          : "s"
      },
      {
         "id"            : 351,
         "name"          : "comp_timebase_set_frac_seconds_val",
         "comment"       : "Set FRACTION_SECONDS value of NTP UTC timestamp",
         "public"        : true,
         "level"         : 0,
         "argument"      : "fraction_of_second"
      }
   ]
}

