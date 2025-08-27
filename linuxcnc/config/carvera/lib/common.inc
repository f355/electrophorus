[EMC]
VERSION = 1.1
MACHINE = Carvera Air
DEBUG = 0x00000000

[DISPLAY]
DISPLAY = qtvcp qtdragon
TITLE = Carvera Air
ICON = silver_dragon.png

# qtdragon saves most preference to this file
PREFERENCE_FILE_PATH = WORKINGFOLDER/qtdragon.pref

# min/max percentage overrides allowed in qtdragon 1 = 100%
MAX_FEED_OVERRIDE = 1.2
MIN_SPINDLE_0_OVERRIDE = 0.5
MAX_SPINDLE_0_OVERRIDE = 1.2

# manual spindle speed will start at this RPM
DEFAULT_SPINDLE_0_SPEED = 10000

# spindle up/down increment in RPM
SPINDLE_INCREMENT = 200

# min max apindle speed manually allowed
MIN_SPINDLE_0_SPEED = 1000
MAX_SPINDLE_0_SPEED = 13000

# max spindle power in Watts
MAX_SPINDLE_POWER = 200

# min/max/default jog velocities in qtdragon in units/sec
MIN_LINEAR_VELOCITY = 0
MAX_LINEAR_VELOCITY = 50.00
DEFAULT_LINEAR_VELOCITY = 50.00

DEFAULT_ANGULAR_VELOCITY = 10
MIN_ANGULAR_VELOCITY = 0.01
MAX_ANGULAR_VELOCITY = 40

# incremental jog step length options
INCREMENTS = 10 mm, 1.0 mm, 0.10 mm, 0.01 mm, 1.0 inch, 0.1 inch, 0.01 inch

CYCLE_TIME = 100
INTRO_GRAPHIC = silver_dragon.png
INTRO_TIME = 2

# default program search path
PROGRAM_PREFIX = ~/linuxcnc/nc_files

# NGCGUI subroutine path.
# Thr path must also  be in [RS274NGC] SUBROUTINE_PATH
NGCGUI_SUBFILE_PATH = ~/linuxcnc/nc_files/ngcgui_lib/
# pre selected programs tabs
# specify filenames only, files must be in the NGCGUI_SUBFILE_PATH
NGCGUI_SUBFILE = slot.ngc
NGCGUI_SUBFILE = qpocket.ngc

# qtdragon saves MDI cxommands to this file
MDI_HISTORY_FILE = mdi_history.dat
# qtdragon saves rnning logs to this file
LOG_FILE = qtdragon.log

[MDI_COMMAND_LIST]
# for macro buttons on main oage up to 10 possible
MDI_COMMAND = G53 G0 Z0;X0 Y0;Z0, Goto\nUser\nZero
MDI_COMMAND = G53 G0 Z0;G53 G0 X0 Y0,Goto\nMachn\nZero

[FILTER]
# Controls what programs are shown inqtdragon file manager
PROGRAM_EXTENSION = .ngc,.nc,.tap G-Code File (*.ngc,*.nc,*.tap)
PROGRAM_EXTENSION = .png,.gif,.jpg Greyscale Depth Image
PROGRAM_EXTENSION = .py Python Script

# specifies what special 'filter' programs runs based on program ending
png = image-to-gcode
gif = image-to-gcode
jpg = image-to-gcode
py = python3

[EMCIO]
EMCIO = io
CYCLE_TIME = 0.100
TOOL_TABLE = tool.tbl

[RS274NGC]
# motion controller saves parameters to this file
PARAMETER_FILE = qtdragon.var

# start up G/M codes when first loaded
RS274NGC_STARTUP_CODE = G17 G21 G40 G43H0 G54 G64P0.0127 G80 G90 G94 G97 M5 M9

# subroutine/remap path list
SUBROUTINE_PATH = ../../../../nc_files/probe/basic_probe/macros:~/linuxcnc/nc_files/examples/ngcgui_lib:~/linuxcnc/nc_files/examples/ngcgui_lib/utilitysubs:~/linuxcnc/nc_files/examples/ngcgui_lib:~/linuxcnc/nc_files/examples/ngcgui_lib/utilitysubs:~/linuxcnc/nc_files/examples/probe/basic_probe/macros:~/linuxcnc/nc_files/examples/remap-subroutines:~/linuxcnc/nc_files/examples/ngcgui_lib/remap_lib

# on abort, this ngc file is called. required for basic/versa probe
ON_ABORT_COMMAND = O <on_abort> call

[EMCMOT]
EMCMOT = motmod
SERVO_PERIOD = 1000000
COMM_TIMEOUT = 1.0
COMM_WAIT = 0.010
BASE_PERIOD = 0
#NUM_AIO = 12

[TASK]
TASK = milltask
CYCLE_TIME = 0.010

[HALUI]
# no content

[PROBE]
USE_PROBE = versaprobe

[TOOL_CHANGE]
X = 0
Y = 0
Z = 0

[CHANGE_POSITION]
X = 0
Y = 0
Z = 0

[VERSA_TOOLSETTER]
X = -6
Y = -3.50
Z = -20
Z_MAX_CLEAR = 0
MAXPROBE = -20

[THERM_POWER]
R0 = 100000
R_OTHER = 4700
BETA = 3950
T0 = 25
V_TOTAL = 65535

[THERM_SPINDLE]
R0 = 100000
R_OTHER = 4700
BETA = 3950
T0 = 25
V_TOTAL = 65535
