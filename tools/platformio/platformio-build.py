#  Copyright (c) 2023
#  Baracoda (SA) All rights reserved, proprietary and confidential
#  Copying this file via any medium without the prior written consent
#  of Baracoda (SA) is strictly prohibited.
#
#  This copyright notice should not be removed

import json
import sys
import time

from os.path import isdir, join

# Init
env = DefaultEnvironment()
platform = env.PioPlatform()
board_config = env.BoardConfig()
board_name = env.subst("$BOARD")

COMPONENTS = []
FRAMEWORK_DIR = platform.get_package_dir("framework-bl-iot-sdk")
assert isdir(FRAMEWORK_DIR)

# Read sdk-data.json
try:
    SDKDATA = json.load(open(join(FRAMEWORK_DIR, 'sdk-data.json')))
except ValueError as err:
    sys.exit("Error reading " + join(FRAMEWORK_DIR, "sdk-data.json"))

# Determine chip name
bl_chipname = board_config.get("build.chipname")
bl_stack_size = board_config.get("build.stack_size")
bl_cache_size = board_config.get("build.cache_size")
bl_mcu = board_config.get("build.mcu")
bl_debug_print = board_config.get("build.debug_print", False)
bl_stack = board_config.get("build.ble_stack", "m1s1p")


# set BL702L BLE stack
SDKDATA['components']["bl702l_ble"]["source_dir"] = SDKDATA['components']["bl702l_ble"]["source_dir"].replace("<ble_stack>", bl_stack)
library_dirs = []
for library_dir in SDKDATA['components']["bl702l_ble"]["library_dirs"]:
    library_dirs.append(library_dir.replace("<ble_stack>", bl_stack))
SDKDATA['components']["bl702l_ble"]["library_dirs"] = library_dirs
link_libraries = []
for link_library in SDKDATA['components']["bl702l_ble"]["link_libraries"]:
    link_libraries.append(link_library.replace("<ble_stack>", bl_stack))
SDKDATA['components']["bl702l_ble"]["link_libraries"] = link_libraries


def debug_print(*argv):
    if bl_debug_print == True:
        print(argv)

def get_list(data):
    if isinstance(data, list):
        return data
    elif isinstance(data, str):
        return [data]
    else:
        return []


def resolve_component_dependencies(COMPONENTS, component):
    if component in SDKDATA['components']:
        dependencies = get_list(SDKDATA['components'][component].get('depends'))
        COMPONENTS = list(set(COMPONENTS + dependencies))
        for dep in dependencies:
            COMPONENTS = resolve_component_dependencies(COMPONENTS, dep)
    return list(set(COMPONENTS))


# Get build components
COMPONENTS = SDKDATA['sdk']['defaults']['components']

# automatically add chip low level compoments
COMPONENTS.append(bl_mcu.lower())
COMPONENTS.append(f"{bl_mcu.lower()}_hosal")

# scan compoments
for component in COMPONENTS:
    COMPONENTS = resolve_component_dependencies(COMPONENTS, component)
COMPONENTS.sort()

for component in COMPONENTS:
    if not component in SDKDATA['components']:
        print(f"component '{component}' wanted for build but not defined")
        sys.exit(1)


# Print Info
print("BOUFFALO SDK:")
print(" - Version: " + SDKDATA['sdk']['version'])
print(" - Chip: " + bl_chipname)
print(" - Stack: " + bl_stack)
print(" - Components: " + ", ".join(COMPONENTS))

#
# Setup Default Build Env
#
include_dirs = []
library_dirs = []
link_libraries = []
defines = get_list(SDKDATA['sdk']['defaults'].get('defines'))
ccflags_priv = get_list(SDKDATA['sdk']['defaults'].get('ccflags_priv'))

# iterate through default include dirs and prepend framework path
for x in range(0, len(get_list(SDKDATA['sdk']['defaults'].get('include_dirs')))):
    include_dirs.append(join(FRAMEWORK_DIR, SDKDATA['sdk']['defaults']['include_dirs'][x]))

# iterate through default library dirs and prepend framework path
for x in range(0, len(get_list(SDKDATA['sdk']['defaults'].get('library_dirs')))):
    library_dirs.append(join(FRAMEWORK_DIR, SDKDATA['sdk']['defaults']['library_dirs'][x]))

for x in range(0, len(get_list(SDKDATA['sdk']['defaults'].get('link_libraries')))):
    link_libraries.append(SDKDATA['sdk']['defaults']['link_libraries'][x])

def define_val(txt):
    for x in range(0, len(defines)):
        stmt = defines[x]
        if isinstance(stmt, list):
            if(stmt[0] == txt):
                return stmt[1]
        else:
            if(stmt == txt):
                return True
    return False

def eval_conditionals(branch):
    global defines, include_dirs
    for eval in branch:
        statement = eval.split()
        first_statement = statement[0] 
        if first_statement == 'ifeq':
            svalue = define_val(statement[1])
            if svalue == type(svalue)(statement[2]):
                debug_print("CONDITION MET (" + statement[1] + "):" + str(svalue))
                if 'include_dirs' in branch[eval]:
                    include_dirs.append(join(FRAMEWORK_DIR, branch[eval]['include_dirs'][x]))
                if 'defines' in branch[eval]:
                    defines += branch[eval]['defines']
        elif first_statement == 'ifneq':
            svalue = define_val(statement[1])
            if svalue != type(svalue)(statement[2]):
                debug_print("CONDITION MET (" + statement[1] + "):" + str(svalue))
                if 'include_dirs' in branch[eval]:
                    include_dirs.append(join(FRAMEWORK_DIR, branch[eval]['include_dirs'][x]))
                if 'defines' in branch[eval]:
                    defines += branch[eval]['defines']
        elif first_statement == 'ifdef':
            svalue = define_val(statement[1])
            if svalue != type(svalue)(False):
                debug_print("CONDITION MET (" + statement[1] + "):" + str(svalue))
                if 'include_dirs' in branch[eval]:
                    include_dirs.append(join(FRAMEWORK_DIR, branch[eval]['include_dirs'][x]))
                if 'defines' in branch[eval]:
                    defines += branch[eval]['defines']
        elif first_statement == 'ifndef':
            svalue = define_val(statement[1])
            if svalue == type(svalue)(False):
                debug_print("CONDITION MET (" + statement[1] + "):" + str(svalue))
                if 'include_dirs' in branch[eval]:
                    include_dirs.append(join(FRAMEWORK_DIR, branch[eval]['include_dirs'][x]))
                if 'defines' in branch[eval]:
                    defines += branch[eval]['defines']
        else:
            print(f"WARNING: invalid conditional statement ({first_statement})")


# add package specific includes and definitions
for i in range(len(COMPONENTS)):
    # select specific hosal build
    if COMPONENTS[i] in SDKDATA['components']:
        defines += get_list(SDKDATA['components'][COMPONENTS[i]].get('defines'))
        for x in range(0, len(get_list(SDKDATA['components'][COMPONENTS[i]].get('include_dirs')))):
            include_dirs.append(join(FRAMEWORK_DIR, SDKDATA['components'][COMPONENTS[i]]['include_dirs'][x]))
        for x in range(0, len(get_list(SDKDATA['components'][COMPONENTS[i]].get('library_dirs')))):
            library_dirs.append(join(FRAMEWORK_DIR, SDKDATA['components'][COMPONENTS[i]]['library_dirs'][x]))
        link_libraries += get_list(SDKDATA['components'][COMPONENTS[i]].get('link_libraries'))

        # Evaluate conditionals
        if 'conditionals' in SDKDATA['components'][COMPONENTS[i]]:
            eval_conditionals(SDKDATA['components'][COMPONENTS[i]]['conditionals'])

# automatically redefine configs
redefines = []
for define in defines:
    if isinstance(define, list):
        if define[0].startswith('CONFIG_'):
           redefines.append([define[0].replace('CONFIG_', ''), define[1]]) 
    else:
        if define.startswith('CONFIG_'):
           redefines.append(define.replace('CONFIG_', '')) 

# find em size
bl_em_size = redefines
for define in defines:
    if isinstance(define, list):
        if define[0] == "CONFIG_EM_SIZE":
           bl_em_size = int(define[1]) * 1024


env.Append(
    ASFLAGS=["-x", "assembler-with-cpp"],
    CFLAGS=[
        "-std=gnu17"
        ],
    CXXFLAGS=[
        "-std=gnu++17",
        "-nostdlib",
        "-fms-extensions",
        "-ffunction-sections",
        "-fdata-sections",
        "-Wall",
        "-Wchar-subscripts",
        "-Wformat",
        "-Winit-self",
        "-Wignored-qualifiers",
        "-Wswitch-default",
        "-Wunused",
        "-Wundef",
        "-fno-rtti",
        "-fno-exceptions",
        "-fno-use-cxa-atexit"
    ],
    CCFLAGS=[
        "-Os",
        "-Wall",  # show warnings
        "-march=%s" % board_config.get("build.march"),
        "-mabi=%s" % board_config.get("build.mabi"),

        "-MMD", 
        "-MP",
        "-ffreestanding",
        "-ffunction-sections",
        "-fdata-sections",
        "-fstrict-volatile-bitfields",
        "-fcommon",
        "-fno-omit-frame-pointer",
        "-gdwarf",
        
        "-Wno-unused-variable",
        "-Wno-unused-function",
        "-Wno-format",
        "-Wno-discarded-qualifiers",
        "-Wno-strict-aliasing",
        "-Wno-array-parameter"

    ],
    CPPDEFINES = defines + redefines + [
        "ARCH_RISCV",
        "_GNU_SOURCE",
        ("__int64_t_defined", 1),
        ("BL_SDK_VER", "\\\"" + f"{SDKDATA['sdk']['version']}" + "\\\""),
        ("BL_SDK_STDDRV_VER", "\\\"" + f"{SDKDATA['sdk']['version']}" + "\\\""),
        ("BL_SDK_STDCOM_VER", "\\\"" + f"{SDKDATA['sdk']['version']}" + "\\\""),
        ("BL_SDK_RF_VER", "\\\"" + f"{SDKDATA['sdk']['rf_ver']}" + "\\\""),
        bl_chipname.upper(),
        ("BL_CHIP_NAME", bl_chipname.upper()),
        ("BFLB_COREDUMP_BINARY_ID", time.time() // 86400 * 86400), # round by day so we don't rebuild every time
        '__FILENAME__=(__builtin_strrchr(\\"/\\"__FILE__, \'/\') + 1)',
        "CONF_USER_"+bl_chipname.upper()
    ],
    CPPPATH = include_dirs + [
    ],
    LINKFLAGS=[
        "-Os",
        "-march=%s" % board_config.get("build.march"),
        "-mabi=%s" % board_config.get("build.mabi"),
        "-nostartfiles",
        "-Wl,--defsym=LD_MAX_SIZE=%d" % board_config.get("upload.maximum_size"),
        "-Wl,--defsym=LD_MAX_DATA_SIZE=%d" % board_config.get("upload.maximum_ram_size"),
        "-Xlinker","-Map="+join("$BUILD_DIR", "firmware.map"),
        "-u _printf_float",
        "-Wl,--defsym=__stack_size=%d" % bl_stack_size,
        "-Wl,--defsym=__CACHE_SIZE=%d" % bl_cache_size,
        "-Wl,--defsym=__EM_SIZE=%d" % bl_em_size,
        "-Wl,--gc-sections",
        "-Wl,--no-warn-rwx-segments",
        "-Wl,-static",
        "-Wl,-EL",
    ],
    LIBS= list(set(link_libraries)) + [
        "m"
    ],
    LIBPATH= library_dirs + [
    ],
)

#
# Linker requires preprocessing with correct RAM|ROM sizes
#
linker_scripts = []
for script in board_config.get("build.linker_script").split('\n'):
    if script != "":
        linker_scripts.append(("-T", script))

env.Prepend(LINKFLAGS=linker_scripts)

#
# Process configuration flags
#

# copy CCFLAGS to ASFLAGS (-x assembler-with-cpp mode)
env.Append(ASFLAGS=env.get("CCFLAGS", [])[:])

#
# Target: Build Core Library
#
libs = []

def component_conditional_source_filter(branch):
    res = []
    for eval in branch:
        statement = eval.split()
        first_statement = statement[0] 
        if first_statement == 'ifeq':
            svalue = define_val(statement[1])
            if svalue == type(svalue)(statement[2]):
                debug_print("CONDITION MET (" + statement[1] + "):" + str(svalue))
                if 'source_filter' in branch[eval]:
                    if isinstance(branch[eval]['source_filter'], list):
                        res.append(" ".join(branch[eval]['source_filter']))
                    else:
                        res.append(branch[eval]['source_filter'])
        elif first_statement == 'ifneq':
            svalue = define_val(statement[1])
            if svalue != type(svalue)(statement[2]):
                debug_print("CONDITION MET (" + statement[1] + "):" + str(svalue))
                if 'source_filter' in branch[eval]:
                    if isinstance(branch[eval]['source_filter'], list):
                        res.append(" ".join(branch[eval]['source_filter']))
                    else:
                        res.append(branch[eval]['source_filter'])
        elif first_statement == 'ifdef':
            svalue = define_val(statement[1])
            if svalue != type(svalue)(False):
                debug_print("CONDITION MET (" + statement[1] + "):" + str(svalue))
                if 'source_filter' in branch[eval]:
                    if isinstance(branch[eval]['source_filter'], list):
                        res.append(" ".join(branch[eval]['source_filter']))
                    else:
                        res.append(branch[eval]['source_filter'])
        elif first_statement == 'ifndef':
            svalue = define_val(statement[1])
            if svalue == type(svalue)(False):
                debug_print("CONDITION MET (" + statement[1] + "):" + str(svalue))
                if 'source_filter' in branch[eval]:
                    if isinstance(branch[eval]['source_filter'], list):
                        res.append(" ".join(branch[eval]['source_filter']))
                    else:
                        res.append(branch[eval]['source_filter'])
        else:
            print(f"WARNING: invalid component conditional statement ({first_statement})")

    return res

# Iterate through components definitions
def find_component_conf(name):
    result = None
    for component_x in SDKDATA['components']:
        if name == component_x:
            result = SDKDATA['components'][component_x]
            break
    return result

# Iterate through included components and build libraries
for x in COMPONENTS:
    component = find_component_conf(x)
    if component is None:
        print("***WARNING: Undefined component (" + x + ")")
        continue

    # iterate through default include dirs and prepend framework path
    include_dirs_priv = []
    for y in range(0, len(get_list(component.get('include_dirs_priv')))):
        include_dirs_priv.append(join(FRAMEWORK_DIR, component['include_dirs_priv'][y]))

    # Clone and set up component build env
    env_c = env.Clone()
    env_c.Append(CPPPATH=include_dirs_priv)
    env_c.Append(CCFLAGS=ccflags_priv)

    # Evaluate conditionals  
    conditional_source_filter = []
    if 'conditionals' in component:
        conditional_source_filter = component_conditional_source_filter(component['conditionals'])

    # Eval source filter
    source_filter = get_list(component.get('source_filter')) + conditional_source_filter

    # Build library
    libs.append(env_c.BuildLibrary(join("$BUILD_DIR", x), join(FRAMEWORK_DIR, component['source_dir']), " ".join(source_filter)))

env.Prepend(LIBS=libs)

