import qbs

MachinekitApplication {
    name: "UNI-PRINT-3D"
    halFiles: ["UNI-PRINT-3D.hal",
               "velocity-extruding.hal"]
    configFiles: ["UNI-PRINT-3D.ini"]
    bbioFiles: ["paralell_cape3.bbio"]
    otherFiles: ["tool.tbl", "subroutines"]
    compFiles: ["led_dim.comp", "thermistor_check.comp"]
    linuxcncIni: "UNI-PRINT-3D.ini"
    //display: "thinkpad.local:0.0"
}
